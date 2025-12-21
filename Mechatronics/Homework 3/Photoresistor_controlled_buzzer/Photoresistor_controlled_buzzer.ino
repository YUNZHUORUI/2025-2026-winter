// 光敏电阻接 A0，蜂鸣器接 D3
const int LDR_PIN = A0;
const int BUZZER_PIN = 3;

// 你可以根据实验环境调整这两个阈值
int darkThreshold = 200;   // 阴暗时低电压
int brightThreshold = 750; // 明亮时高电压

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(LDR_PIN); // 读取光敏电阻电压值（0~1023）
  Serial.println(sensorValue);           // 串口输出方便调试

  // 判断亮度范围
  if (sensorValue < darkThreshold) {
    // 太暗 → 输出 100 Hz 方波
    tone(BUZZER_PIN, 100);
  } 
  else if (sensorValue > brightThreshold) {
    // 太亮 → 输出 250 Hz 方波
    tone(BUZZER_PIN, 250);
  } 
  else {
    // 中间亮度 → 静音
    noTone(BUZZER_PIN);
  }

  delay(100); // 每 100ms 更新一次
}
