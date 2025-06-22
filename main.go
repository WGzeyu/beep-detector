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
	BITS_PER_SAMPLE = 16    // 位深度
	BUFFER_SIZE     = 2048  // 缓冲区大小

	// 频率检测参数
	SCAN_START_FREQ   = 2400.0 // 扫描起始频率
	SCAN_END_FREQ     = 2700.0 // 扫描结束频率
	SCAN_INTERVAL     = 10.0   // 扫描间隔
	FINE_SCAN_RANGE   = 2      // 精确扫描范围倍数
	PRECISE_SCAN_STEP = 1.0    // 精确扫描步长

	// 检测参数
	BEEP_DURATION_MAX = 480 // 蜂鸣声最长时间
	BEEP_DURATION_MIN = 180 // 蜂鸣声最短时间
	ENERGY_THRESHOLD  = 10  // 归一化触发能量阈值
	TRIGGER_THRESHOLD = 3   // 连续几帧以触发判定（防抖）
	HARMONIC_RATIO    = 0.2 // 次频率触发倍率

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

type FrequencyResult struct {
	Frequency float64
	Energy    float64
}

type BeepDetector struct {
	udpConn        *net.UDPConn
	isBeeping      bool
	hWaveIn        uintptr
	buffer1        []byte
	buffer2        []byte
	header1        WAVEHDR
	header2        WAVEHDR
	bufferDuration time.Duration // 缓冲区时长
	triggerCount   int           // 触发计数
	silenceCount   int           // 静默计数
	triggerTime    time.Time     // 首次触发时间
	endTime        time.Time     // 结束触发时间

	// 频率缓存
	frequencyCache []float64

	// 预计算的Goertzel系数，避免重复计算
	goertzelCoeffs map[float64]float64
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
		frequencyCache: make([]float64, 0, 100),
		goertzelCoeffs: make(map[float64]float64),
	}

	// 预计算常用频率的Goertzel系数
	detector.precomputeGoertzelCoeffs()

	return detector, nil
}

func (bd *BeepDetector) precomputeGoertzelCoeffs() {
	// 预计算扫描频率范围的系数
	for freq := SCAN_START_FREQ; freq <= SCAN_END_FREQ; freq += SCAN_INTERVAL {
		bd.precomputeCoeff(freq)
		bd.precomputeCoeff(freq * 2) // 二次谐波
	}

	// 预计算精确扫描可能用到的系数
	for freq := SCAN_START_FREQ - SCAN_INTERVAL*FINE_SCAN_RANGE; freq <= SCAN_END_FREQ+SCAN_INTERVAL*FINE_SCAN_RANGE; freq += PRECISE_SCAN_STEP {
		bd.precomputeCoeff(freq)
	}
}

func (bd *BeepDetector) precomputeCoeff(targetFreq float64) {
	if _, exists := bd.goertzelCoeffs[targetFreq]; exists {
		return
	}

	k := int(0.5 + float64(BUFFER_SIZE)*targetFreq/SAMPLE_RATE)
	w := 2.0 * math.Pi * float64(k) / float64(BUFFER_SIZE)
	coeff := 2.0 * math.Cos(w)
	bd.goertzelCoeffs[targetFreq] = coeff
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

// 优化的Goertzel算法 - 使用预计算的系数
func (bd *BeepDetector) goertzelDetect(samples []int16, targetFreq float64) float64 {
	n := len(samples)
	if n == 0 {
		return 0
	}

	// 获取预计算的系数
	coeff, exists := bd.goertzelCoeffs[targetFreq]
	if !exists {
		bd.precomputeCoeff(targetFreq)
		coeff = bd.goertzelCoeffs[targetFreq]
	}

	// 计算sin/cos用于最终幅度计算
	k := int(0.5 + float64(n)*targetFreq/SAMPLE_RATE)
	w := 2.0 * math.Pi * float64(k) / float64(n)
	cosw := math.Cos(w)
	sinw := math.Sin(w)

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

// 扫描频率范围找到能量最高的基频
func (bd *BeepDetector) scanBaseFrequency(samples []int16, totalEnergy float64) *FrequencyResult {
	var bestFreq float64
	var bestEnergy float64

	// 粗扫描：按间隔扫描整个频率范围
	for freq := SCAN_START_FREQ; freq <= SCAN_END_FREQ; freq += SCAN_INTERVAL {
		energy := bd.goertzelDetect(samples, freq)
		if totalEnergy > 0 {
			energy /= totalEnergy
		}

		if energy > bestEnergy {
			bestEnergy = energy
			bestFreq = freq
		}
	}

	// 检查最佳基频是否达到阈值
	if bestEnergy < ENERGY_THRESHOLD {
		return nil
	}

	return &FrequencyResult{
		Frequency: bestFreq,
		Energy:    bestEnergy,
	}
}

// 检查二次谐波
func (bd *BeepDetector) checkHarmonic(samples []int16, baseFreq float64, totalEnergy float64) bool {
	harmonicFreq := baseFreq * 2

	// 在基频二次谐波附近扫描
	scanRange := SCAN_INTERVAL * FINE_SCAN_RANGE * 2
	var maxHarmonicEnergy float64

	for freq := harmonicFreq - scanRange; freq <= harmonicFreq+scanRange; freq += SCAN_INTERVAL {
		energy := bd.goertzelDetect(samples, freq)
		if totalEnergy > 0 {
			energy /= totalEnergy
		}

		if energy > maxHarmonicEnergy {
			maxHarmonicEnergy = energy
		}
	}

	return maxHarmonicEnergy > ENERGY_THRESHOLD*HARMONIC_RATIO
}

// 精确扫描确定准确频率
func (bd *BeepDetector) preciseFrequencyScan(samples []int16, roughFreq float64, totalEnergy float64) float64 {
	var bestFreq = roughFreq
	var bestEnergy float64

	// 在粗扫描结果附近进行1Hz精度的扫描
	scanRange := SCAN_INTERVAL
	for freq := roughFreq - scanRange; freq <= roughFreq+scanRange; freq += PRECISE_SCAN_STEP {
		energy := bd.goertzelDetect(samples, freq)
		if totalEnergy > 0 {
			energy /= totalEnergy
		}

		if energy > bestEnergy {
			bestEnergy = energy
			bestFreq = freq
		}
	}

	return bestFreq
}

// 检测是否为蜂鸣声
func (bd *BeepDetector) detectBeep(audioData []byte) (bool, float64) {
	// 将字节数据转换为int16采样
	samples := make([]int16, len(audioData)/2)
	for i := 0; i < len(samples); i++ {
		samples[i] = int16(audioData[i*2]) | (int16(audioData[i*2+1]) << 8)
	}

	// 计算总能量（用于归一化）
	var totalEnergy float64
	for _, sample := range samples {
		totalEnergy += float64(sample) * float64(sample)
	}

	// 1. 扫描基频
	baseResult := bd.scanBaseFrequency(samples, totalEnergy)
	if baseResult == nil {
		return false, 0
	}

	// 2. 检查二次谐波
	if !bd.checkHarmonic(samples, baseResult.Frequency, totalEnergy) {
		return false, 0
	}

	// 3. 精确扫描确定准确频率
	preciseFreq := bd.preciseFrequencyScan(samples, baseResult.Frequency, totalEnergy)

	// 调试输出
	log.Printf("检测到蜂鸣: 基频=%.1fHz, 精确频率=%.1fHz, 能量=%.6f",
		baseResult.Frequency, preciseFreq, baseResult.Energy)

	return true, preciseFreq
}

// 发送UDP消息 - 新格式：状态码 + 频率 + 持续时间 + 时间戳
func (bd *BeepDetector) sendBinaryUDPMessage(code int16, frequency *float64, duration *time.Duration, timestamp *time.Time) {
	buf := make([]byte, 0, 18) // 最大长度：2+4+4+8

	// 2字节：类型码
	codeBytes := make([]byte, 2)
	binary.LittleEndian.PutUint16(codeBytes, uint16(code))
	buf = append(buf, codeBytes...)

	// 4字节：频率（可选）
	if frequency != nil {
		freqBytes := make([]byte, 4)
		binary.LittleEndian.PutUint32(freqBytes, uint32(*frequency*100000)) // 频率×100000
		buf = append(buf, freqBytes...)
	}

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
		log.Printf("已发送UDP二进制消息: code=%d, frequency=%v, duration=%v, timestamp=%v",
			code,
			func() any {
				if frequency != nil {
					return fmt.Sprintf("%.1f", *frequency)
				}
				return "N/A"
			}(),
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

// 计算频率缓存的平均值
func (bd *BeepDetector) getAverageFrequency() float64 {
	if len(bd.frequencyCache) == 0 {
		return 0
	}

	var sum float64
	for _, freq := range bd.frequencyCache {
		sum += freq
	}

	return sum / float64(len(bd.frequencyCache))
}

// 处理音频数据的新逻辑
func (bd *BeepDetector) processAudioNew(audioData []byte) {
	now := time.Now()
	isBeepDetected, detectedFreq := bd.detectBeep(audioData)

	if isBeepDetected {
		bd.triggerCount++
		bd.silenceCount = 0

		// 将检测到的频率加入缓存
		bd.frequencyCache = append(bd.frequencyCache, detectedFreq)

		if bd.triggerCount == 1 && !bd.isBeeping {
			bd.triggerTime = now
		}

		if bd.triggerCount >= TRIGGER_THRESHOLD && !bd.isBeeping {
			bd.isBeeping = true
			log.Printf("检测到蜂鸣声开始: %v", now.Format("15:04:05.000"))
		}
	} else {
		bd.silenceCount++
		bd.triggerCount = 0

		// 如果当前是非蜂鸣触发状态，直接清空频率缓存
		if !bd.isBeeping {
			bd.frequencyCache = bd.frequencyCache[:0]
		}

		if bd.silenceCount == 1 && bd.isBeeping {
			bd.endTime = now
		}

		if bd.silenceCount >= TRIGGER_THRESHOLD && bd.isBeeping {
			duration := bd.endTime.Sub(bd.triggerTime)
			bd.isBeeping = false
			triggerMaxTime := time.Duration(BEEP_DURATION_MAX) * time.Millisecond
			triggerMinTime := time.Duration(BEEP_DURATION_MIN) * time.Millisecond

			// 计算平均频率
			avgFreq := bd.getAverageFrequency()

			// 检查持续时间是否符合预期（允许一定误差）
			if duration >= triggerMinTime && duration <= triggerMaxTime {
				bd.sendBinaryUDPMessage(5, &avgFreq, &duration, &bd.triggerTime) // 发送蜂鸣消息
				log.Printf("蜂鸣声检测完成: 持续时间 %dms, 平均频率 %.1fHz",
					duration.Milliseconds(), avgFreq)
			} else {
				log.Printf("蜂鸣声持续时间不匹配: %dms (期望约%d-%dms), 平均频率 %.1fHz",
					duration.Milliseconds(), triggerMinTime.Milliseconds(), triggerMaxTime.Milliseconds(),
					avgFreq)
				bd.sendBinaryUDPMessage(105, &avgFreq, &duration, &bd.triggerTime) // 发送蜂鸣时间不匹配消息
			}

			// 清空频率缓存
			bd.frequencyCache = bd.frequencyCache[:0]
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
		bd.sendBinaryUDPMessage(201, nil, nil, nil)
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
		bd.sendBinaryUDPMessage(202, nil, nil, nil)
		return fmt.Errorf("waveInPrepareHeader 失败 (buffer1)，错误代码: %d", ret1)
	}

	ret2, _, _ := waveInPrepareHeader.Call(bd.hWaveIn, uintptr(unsafe.Pointer(&bd.header2)), unsafe.Sizeof(bd.header2))
	if ret2 != 0 {
		bd.sendBinaryUDPMessage(203, nil, nil, nil)
		return fmt.Errorf("waveInPrepareHeader 失败 (buffer2)，错误代码: %d", ret2)
	}

	// 添加缓冲区到队列
	ret3, _, _ := waveInAddBuffer.Call(bd.hWaveIn, uintptr(unsafe.Pointer(&bd.header1)), unsafe.Sizeof(bd.header1))
	if ret3 != 0 {
		bd.sendBinaryUDPMessage(204, nil, nil, nil)
		return fmt.Errorf("waveInAddBuffer 失败 (buffer1)，错误代码: %d", ret3)
	}

	ret4, _, _ := waveInAddBuffer.Call(bd.hWaveIn, uintptr(unsafe.Pointer(&bd.header2)), unsafe.Sizeof(bd.header2))
	if ret4 != 0 {
		bd.sendBinaryUDPMessage(205, nil, nil, nil)
		return fmt.Errorf("waveInAddBuffer 失败 (buffer2)，错误代码: %d", ret4)
	}

	// 开始录音
	ret, _, _ = waveInStart.Call(bd.hWaveIn)
	if ret != 0 {
		bd.sendBinaryUDPMessage(206, nil, nil, nil)
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
	log.Printf("扫描频率范围: %.0f-%.0fHz (间隔%.0fHz)",
		SCAN_START_FREQ, SCAN_END_FREQ, SCAN_INTERVAL)
	log.Printf("UDP上报地址: %s", UDP_PORT)
	log.Println("正在启动音频录制...")

	// 开始录音
	err = detector.startRecording()
	if err != nil {
		log.Fatalf("启动录音失败: %v", err)
		detector.sendBinaryUDPMessage(222, nil, nil, nil) // 发送错误消息
	}

	log.Println("音频录制已启动，开始检测蜂鸣声...")
	log.Println("按 Ctrl+C 停止...")
	detector.sendBinaryUDPMessage(100, nil, nil, nil) // 发送启动消息

	// 主循环 - 轮询检查缓冲区状态
	go func() {
		for {
			// 检查第一个缓冲区
			if detector.header1.Flags&WHDR_DONE != 0 {
				detector.processAudioNew(detector.buffer1[:detector.header1.BytesRecorded])

				// 重置缓冲区状态 - 只清除WHDR_DONE标志，保留其他标志
				detector.header1.Flags &= ^uint32(WHDR_DONE)
				detector.header1.BytesRecorded = 0

				// 重新添加缓冲区到队列
				ret, _, _ := waveInAddBuffer.Call(detector.hWaveIn, uintptr(unsafe.Pointer(&detector.header1)), unsafe.Sizeof(detector.header1))
				if ret != 0 {
					log.Printf("重新添加缓冲区1失败，错误代码: %d", ret)
					detector.sendBinaryUDPMessage(231, nil, nil, nil) // 发送错误消息
				}
			}

			// 检查第二个缓冲区
			if detector.header2.Flags&WHDR_DONE != 0 {
				detector.processAudioNew(detector.buffer2[:detector.header2.BytesRecorded])

				// 重置缓冲区状态 - 只清除WHDR_DONE标志，保留其他标志
				detector.header2.Flags &= ^uint32(WHDR_DONE)
				detector.header2.BytesRecorded = 0

				// 重新添加缓冲区到队列
				ret, _, _ := waveInAddBuffer.Call(detector.hWaveIn, uintptr(unsafe.Pointer(&detector.header2)), unsafe.Sizeof(detector.header2))
				if ret != 0 {
					log.Printf("重新添加缓冲区2失败，错误代码: %d", ret)
					detector.sendBinaryUDPMessage(232, nil, nil, nil) // 发送错误消息
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
	detector.sendBinaryUDPMessage(300, nil, nil, nil) // 发送关闭消息
	log.Println("正在关闭...")
}
