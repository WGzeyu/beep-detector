package main

import (
	"encoding/binary"
	"fmt"
	"log"
	"math"
	"net"
	"os"
	"os/signal"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

const (
	// 音频参数
	SAMPLE_RATE     = 44100 // 采样率
	CHANNELS        = 1     // 单声道
	BITS_PER_SAMPLE = 16    // 16位采样
	BUFFER_SIZE     = 2048  // 缓冲区大小

	// 检测参数
	TARGET_FREQ_1 = 2620 // 主频率 2.62kHz
	TARGET_FREQ_2 = 5240 // 次频率 5.24kHz
	//FREQ_TOLERANCE    = 100  // 频率容差 ±100Hz
	BEEP_DURATION     = 320 // 蜂鸣声持续时间 375ms
	ENERGY_THRESHOLD  = 5   // 归一化能量阈值，需要根据实际情况调整
	TRIGGER_THRESHOLD = 3   // 连续达此帧数后触发
	HARMONIC_RATIO    = 0.1 //次频率触发倍率

	// UDP上报
	UDP_PORT = "127.0.0.1:25562"

	// Windows API 常量
	WAVE_FORMAT_PCM = 1
	CALLBACK_NULL   = 0

	// WAVEHDR Flags
	WHDR_DONE      = 0x00000001
	WHDR_PREPARED  = 0x00000002
	WHDR_BEGINLOOP = 0x00000004
	WHDR_ENDLOOP   = 0x00000008
	WHDR_INQUEUE   = 0x00000010
)

var (
	winmm = windows.NewLazySystemDLL("winmm.dll")

	waveInOpen            = winmm.NewProc("waveInOpen")
	waveInPrepareHeader   = winmm.NewProc("waveInPrepareHeader")
	waveInUnprepareHeader = winmm.NewProc("waveInUnprepareHeader")
	waveInAddBuffer       = winmm.NewProc("waveInAddBuffer")
	waveInStart           = winmm.NewProc("waveInStart")
	waveInStop            = winmm.NewProc("waveInStop")
	waveInClose           = winmm.NewProc("waveInClose")
	waveInReset           = winmm.NewProc("waveInReset")
)

type WAVEFORMATEX struct {
	FormatTag      uint16
	Channels       uint16
	SamplesPerSec  uint32
	AvgBytesPerSec uint32
	BlockAlign     uint16
	BitsPerSample  uint16
	Size           uint16
}

type WAVEHDR struct {
	Data          uintptr
	BufferLength  uint32
	BytesRecorded uint32
	User          uintptr
	Flags         uint32
	Loops         uint32
	Next          uintptr
	Reserved      uintptr
}

type BeepDetector struct {
	udpConn *net.UDPConn
	//lastBeepTime  time.Time
	isBeeping bool
	//beepStartTime time.Time
	hWaveIn uintptr
	buffer1 []byte
	buffer2 []byte
	header1 WAVEHDR
	header2 WAVEHDR
	//currentBuffer  int
	bufferDuration time.Duration // 新增：每个缓冲区的时长
	triggerCount   int           // 新增：触发计数
	silenceCount   int           // 新增：静默计数
	triggerTime    time.Time     // 新增：首次触发时间
	endTime        time.Time     // 新增：结束触发时间
}

func NewBeepDetector() (*BeepDetector, error) {
	// 初始化UDP连接
	udpAddr, err := net.ResolveUDPAddr("udp", UDP_PORT)
	if err != nil {
		return nil, fmt.Errorf("解析UDP地址失败: %v", err)
	}

	udpConn, err := net.DialUDP("udp", nil, udpAddr)
	if err != nil {
		return nil, fmt.Errorf("创建UDP连接失败: %v", err)
	}
	// 计算缓冲区时长 (毫秒)
	seconds := float64(BUFFER_SIZE) / float64(SAMPLE_RATE)
	bufferDur := time.Duration(int64(seconds * float64(time.Second)))
	detector := &BeepDetector{
		udpConn:        udpConn,
		buffer1:        make([]byte, BUFFER_SIZE*2), // 16位采样，所以乘以2
		buffer2:        make([]byte, BUFFER_SIZE*2),
		bufferDuration: bufferDur,
	}

	return detector, nil
}

func (bd *BeepDetector) Close() {
	if bd.hWaveIn != 0 {
		waveInStop.Call(bd.hWaveIn)
		waveInReset.Call(bd.hWaveIn)

		// 取消准备缓冲区头
		waveInUnprepareHeader.Call(bd.hWaveIn, uintptr(unsafe.Pointer(&bd.header1)), unsafe.Sizeof(bd.header1))
		waveInUnprepareHeader.Call(bd.hWaveIn, uintptr(unsafe.Pointer(&bd.header2)), unsafe.Sizeof(bd.header2))

		waveInClose.Call(bd.hWaveIn)
	}
	if bd.udpConn != nil {
		bd.udpConn.Close()
	}
}

// 高效的频率检测算法 - 使用Goertzel算法
func (bd *BeepDetector) goertzelDetect(samples []int16, targetFreq float64) float64 {
	n := len(samples)
	if n == 0 {
		return 0
	}

	// Goertzel算法参数
	k := int(0.5 + float64(n)*targetFreq/SAMPLE_RATE)
	w := 2.0 * math.Pi * float64(k) / float64(n)
	cosw := math.Cos(w)
	sinw := math.Sin(w)
	coeff := 2.0 * cosw

	var q0, q1, q2 float64

	// Goertzel算法核心循环
	for i := 0; i < n; i++ {
		q0 = coeff*q1 - q2 + float64(samples[i])
		q2 = q1
		q1 = q0
	}

	// 计算幅度
	real := q1 - q2*cosw
	imag := q2 * sinw
	magnitude := real*real + imag*imag

	return magnitude
}

// 检测是否为蜂鸣声
func (bd *BeepDetector) detectBeep(audioData []byte) bool {
	// 将字节数据转换为int16采样
	samples := make([]int16, len(audioData)/2)
	for i := 0; i < len(samples); i++ {
		samples[i] = int16(audioData[i*2]) | (int16(audioData[i*2+1]) << 8)
	}

	// 使用Goertzel算法检测目标频率
	energy1 := bd.goertzelDetect(samples, TARGET_FREQ_1)
	energy2 := bd.goertzelDetect(samples, TARGET_FREQ_2)

	// 计算总能量（用于归一化）
	var totalEnergy float64
	for _, sample := range samples {
		totalEnergy += float64(sample) * float64(sample)
	}

	// 归一化能量
	if totalEnergy > 0 {
		energy1 /= totalEnergy
		energy2 /= totalEnergy
	}

	// 调试输出
	if bd.isBeeping {
		log.Printf("energy1: %.6f, energy2: %.6f, total: %.2f, bufferDuration: %s", energy1, energy2, totalEnergy, bd.bufferDuration)
	}
	// 检测逻辑：主频率能量要足够大，且次频率也要有一定能量
	return energy1 > ENERGY_THRESHOLD && energy2 > ENERGY_THRESHOLD*HARMONIC_RATIO
}

// 发送UDP消息
func (bd *BeepDetector) sendUDPMessage(message string) {
	_, err := bd.udpConn.Write([]byte(message))
	if err != nil {
		log.Printf("发送UDP消息失败: %v", err)
	} else {
		log.Printf("已发送UDP消息: %s", message)
	}
}

// 调用这个替代原先的 sendUDPMessage(message string)
func (bd *BeepDetector) sendBinaryUDPMessage(code int16, duration *time.Duration, timestamp *time.Time) {
	buf := make([]byte, 0, 14)

	// 2字节：类型码
	codeBytes := make([]byte, 2)
	binary.LittleEndian.PutUint16(codeBytes, uint16(code))
	buf = append(buf, codeBytes...)

	// 4字节：持续时间（可选）
	if duration != nil {
		durBytes := make([]byte, 4)
		binary.LittleEndian.PutUint32(durBytes, uint32(duration.Milliseconds()))
		buf = append(buf, durBytes...)
	}

	// 8字节：时间戳（可选）
	if timestamp != nil {
		tsBytes := make([]byte, 8)
		binary.LittleEndian.PutUint64(tsBytes, uint64(timestamp.UnixMilli()))
		buf = append(buf, tsBytes...)
	}

	_, err := bd.udpConn.Write(buf)
	if err != nil {
		log.Printf("发送UDP二进制消息失败: %v", err)
	} else {
		log.Printf("已发送UDP二进制消息: code=%d, duration=%v, timestamp=%v",
			code,
			func() any {
				if duration != nil {
					return duration.Milliseconds()
				}
				return "N/A"
			}(),
			func() any {
				if timestamp != nil {
					return timestamp.UnixMilli()
				}
				return "N/A"
			}(),
		)
	}
}

// 用串口断电监测工具中的逻辑增强鲁棒性
func (bd *BeepDetector) processAudioNew(audioData []byte) {
	now := time.Now()
	isBeepDetected := bd.detectBeep(audioData)

	if isBeepDetected {
		bd.triggerCount++
		bd.silenceCount = 0

		if bd.triggerCount == 1 && !bd.isBeeping {
			bd.triggerTime = now
		}

		if bd.triggerCount >= TRIGGER_THRESHOLD && !bd.isBeeping {
			bd.isBeeping = true
			//bd.beepStartTime = bd.triggerTime
			log.Printf("检测到蜂鸣声开始: %v", now.Format("15:04:05.000"))
		}
	} else {
		bd.silenceCount++
		bd.triggerCount = 0

		if bd.silenceCount == 1 && bd.isBeeping {
			bd.endTime = now
		}

		if bd.silenceCount >= TRIGGER_THRESHOLD && bd.isBeeping {
			duration := bd.endTime.Sub(bd.triggerTime)
			bd.isBeeping = false
			expectedDuration := time.Duration(BEEP_DURATION) * time.Millisecond
			triggerMaxTime := (expectedDuration * 15 / 10) + bd.bufferDuration
			triggerMinTime := (expectedDuration * 5 / 10) - bd.bufferDuration
			// 检查持续时间是否符合预期（允许一定误差）
			if duration >= triggerMinTime && duration <= triggerMaxTime {
				/*message := fmt.Sprintf("BEEP_DETECTED|TIME:%s|DURATION:%dms",
				bd.triggerTime.Format("2006-01-02 15:04:05.000"),
				duration.Milliseconds())*/
				bd.sendBinaryUDPMessage(5, &duration, &bd.triggerTime) //发送蜂鸣消息
				log.Printf("蜂鸣声检测完成: 持续时间 %dms", duration.Milliseconds())
			} else {
				log.Printf("蜂鸣声持续时间不匹配: %dms (期望约%d-%dms)",
					duration.Milliseconds(), triggerMinTime.Milliseconds(), triggerMaxTime.Milliseconds())
				bd.sendBinaryUDPMessage(105, &duration, &bd.triggerTime) //发送蜂鸣时间不匹配消息
			}
		}
	}
}

func (bd *BeepDetector) startRecording() error {
	// 设置音频格式
	format := WAVEFORMATEX{
		FormatTag:      WAVE_FORMAT_PCM,
		Channels:       CHANNELS,
		SamplesPerSec:  SAMPLE_RATE,
		AvgBytesPerSec: SAMPLE_RATE * CHANNELS * BITS_PER_SAMPLE / 8,
		BlockAlign:     CHANNELS * BITS_PER_SAMPLE / 8,
		BitsPerSample:  BITS_PER_SAMPLE,
		Size:           0,
	}

	// 打开音频输入设备
	ret, _, _ := waveInOpen.Call(
		uintptr(unsafe.Pointer(&bd.hWaveIn)),
		^uintptr(0), // WAVE_MAPPER
		uintptr(unsafe.Pointer(&format)),
		0, // callback
		0, // callback instance
		CALLBACK_NULL,
	)

	if ret != 0 {
		bd.sendBinaryUDPMessage(201, nil, nil)
		return fmt.Errorf("waveInOpen 失败，错误代码: %d", ret)
	}

	// 准备缓冲区
	bd.header1.Data = uintptr(unsafe.Pointer(&bd.buffer1[0]))
	bd.header1.BufferLength = uint32(len(bd.buffer1))
	bd.header1.Flags = 0

	bd.header2.Data = uintptr(unsafe.Pointer(&bd.buffer2[0]))
	bd.header2.BufferLength = uint32(len(bd.buffer2))
	bd.header2.Flags = 0

	// 准备缓冲区头
	ret1, _, _ := waveInPrepareHeader.Call(bd.hWaveIn, uintptr(unsafe.Pointer(&bd.header1)), unsafe.Sizeof(bd.header1))
	if ret1 != 0 {
		bd.sendBinaryUDPMessage(202, nil, nil)
		return fmt.Errorf("waveInPrepareHeader 失败 (buffer1)，错误代码: %d", ret1)
	}

	ret2, _, _ := waveInPrepareHeader.Call(bd.hWaveIn, uintptr(unsafe.Pointer(&bd.header2)), unsafe.Sizeof(bd.header2))
	if ret2 != 0 {
		bd.sendBinaryUDPMessage(203, nil, nil)
		return fmt.Errorf("waveInPrepareHeader 失败 (buffer2)，错误代码: %d", ret2)
	}

	// 添加缓冲区到队列
	ret3, _, _ := waveInAddBuffer.Call(bd.hWaveIn, uintptr(unsafe.Pointer(&bd.header1)), unsafe.Sizeof(bd.header1))
	if ret3 != 0 {
		bd.sendBinaryUDPMessage(204, nil, nil)
		return fmt.Errorf("waveInAddBuffer 失败 (buffer1)，错误代码: %d", ret3)
	}

	ret4, _, _ := waveInAddBuffer.Call(bd.hWaveIn, uintptr(unsafe.Pointer(&bd.header2)), unsafe.Sizeof(bd.header2))
	if ret4 != 0 {
		bd.sendBinaryUDPMessage(205, nil, nil)
		return fmt.Errorf("waveInAddBuffer 失败 (buffer2)，错误代码: %d", ret4)
	}

	// 开始录音
	ret, _, _ = waveInStart.Call(bd.hWaveIn)
	if ret != 0 {
		bd.sendBinaryUDPMessage(206, nil, nil)
		return fmt.Errorf("waveInStart 失败，错误代码: %d", ret)
	}

	return nil
}

func main() {
	log.Println("初始化蜂鸣器检测器...")

	// 创建检测器
	detector, err := NewBeepDetector()
	if err != nil {
		log.Fatalf("创建检测器失败: %v", err)
	}
	defer detector.Close()

	log.Printf("蜂鸣器检测器已启动")
	log.Printf("目标频率: %.2fkHz (主), %.2fkHz (次)",
		float64(TARGET_FREQ_1)/1000, float64(TARGET_FREQ_2)/1000)
	log.Printf("UDP上报地址: %s", UDP_PORT)
	log.Println("正在启动音频录制...")

	// 开始录音
	err = detector.startRecording()
	if err != nil {
		log.Fatalf("启动录音失败: %v", err)
		detector.sendBinaryUDPMessage(222, nil, nil) // 发送错误消息
	}

	log.Println("音频录制已启动，开始检测蜂鸣声...")
	log.Println("按 Ctrl+C 停止...")
	detector.sendBinaryUDPMessage(100, nil, nil) // 发送启动消息

	// 主循环 - 轮询检查缓冲区状态
	go func() {
		for {
			// 检查第一个缓冲区
			if detector.header1.Flags&WHDR_DONE != 0 {
				// log.Printf("处理缓冲区1: %d 字节", detector.header1.BytesRecorded)
				detector.processAudioNew(detector.buffer1[:detector.header1.BytesRecorded])

				// 重置缓冲区状态 - 只清除WHDR_DONE标志，保留其他标志
				detector.header1.Flags &= ^uint32(WHDR_DONE)
				detector.header1.BytesRecorded = 0

				// 重新添加缓冲区到队列
				ret, _, _ := waveInAddBuffer.Call(detector.hWaveIn, uintptr(unsafe.Pointer(&detector.header1)), unsafe.Sizeof(detector.header1))
				if ret != 0 {
					log.Printf("重新添加缓冲区1失败，错误代码: %d", ret)
					detector.sendBinaryUDPMessage(231, nil, nil) // 发送错误消息
				}
			}

			// 检查第二个缓冲区
			if detector.header2.Flags&WHDR_DONE != 0 {
				// log.Printf("处理缓冲区2: %d 字节", detector.header2.BytesRecorded)
				detector.processAudioNew(detector.buffer2[:detector.header2.BytesRecorded])

				// 重置缓冲区状态 - 只清除WHDR_DONE标志，保留其他标志
				detector.header2.Flags &= ^uint32(WHDR_DONE)
				detector.header2.BytesRecorded = 0

				// 重新添加缓冲区到队列
				ret, _, _ := waveInAddBuffer.Call(detector.hWaveIn, uintptr(unsafe.Pointer(&detector.header2)), unsafe.Sizeof(detector.header2))
				if ret != 0 {
					log.Printf("重新添加缓冲区2失败，错误代码: %d", ret)
					detector.sendBinaryUDPMessage(232, nil, nil) // 发送错误消息
				}
			}

			// 短暂休眠以降低CPU占用
			time.Sleep(10 * time.Millisecond)
		}
	}()

	// 等待中断信号
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	<-sigChan
	detector.sendBinaryUDPMessage(300, nil, nil) // 发送关闭消息
	log.Println("正在关闭...")
}
