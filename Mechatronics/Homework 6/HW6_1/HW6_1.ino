// 定义输出引脚
const int outputPin = 8; 

// 方波周期 T = 1/50 Hz = 20 ms
// 高电平时间 = 10 ms
// 低电平时间 = 10 ms
const int pulseWidth_ms = 10; 

void setup() {
  // 设置引脚为输出模式
  pinMode(outputPin, OUTPUT);
  // 可选：启动串口用于调试
  // Serial.begin(9600);
}

void loop() {
  // --- 上升沿 (高电平) ---
  // 将输出引脚设置为高电平 (Vhigh ≈ 5V)
  digitalWrite(outputPin, HIGH);
  // 保持高电平 10 毫秒
  delay(pulseWidth_ms); 
  
  // --- 下降沿 (低电平) ---
  // 将输出引脚设置为低电平 (Vlow ≈ 0V)
  digitalWrite(outputPin, LOW);
  // 保持低电平 10 毫秒
  delay(pulseWidth_ms); 
}
