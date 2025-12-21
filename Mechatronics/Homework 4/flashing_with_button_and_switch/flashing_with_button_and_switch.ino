// ================================================================
//                      引脚定义
// ================================================================
const int BUTTON_PIN = 2;   // 按钮输入 (中断)
const int LED_PIN = 8;      // LED 输出
const int SWITCH_PIN = 7;   // SPDT 中间脚

// ================================================================
//                      全局变量
// ================================================================
volatile int freqLevel = 1;       // 当前频率档位 (1~10)
unsigned long lastButtonTime = 0; // 中断防抖
int lastSwitchState = LOW;        // 上一次开关状态
bool needsUpdate = true;          // 是否需要更新频率
int ledState = LOW;               // LED 当前状态
unsigned long previousMillis = 0; // 用于非阻塞闪烁
unsigned long lastSendTime = 0;   // 用于控制发送间隔
int currentFreqToSend = 1;        // 当前要发送的频率数字

// ================================================================
//                      函数声明
// ================================================================
void changeFrequencyISR();
void checkModeChange();
void updateLED();
void printStatus();

// ================================================================
//                      初始化
// ================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(9600);     // 串口监视器
  Serial1.begin(9600);    // TX1输出到示波器 (Pin 18)

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), changeFrequencyISR, FALLING);

  checkModeChange();  // 初始化模式
  printStatus();
}

// ================================================================
//                      主循环
// ================================================================
void loop() {
  checkModeChange();
  updateLED();

  // --- 定期发送当前频率到示波器 ---
  unsigned long now = millis();
  if (now - lastSendTime >= 10) { // 每10ms发送一次 (100Hz)
    Serial1.write(currentFreqToSend);  // 持续发送频率数字的二进制码
    lastSendTime = now;
  }

  // 如果有状态更新（切换开关或按按钮）
  if (needsUpdate) {
    int switchState = digitalRead(SWITCH_PIN);
    float actualFreq = (switchState == HIGH) ? freqLevel * 10.0 : freqLevel;

    // 更新要发送的数字
    currentFreqToSend = (int)round(actualFreq);
    printStatus();
    needsUpdate = false;
  }
}

// ================================================================
//        中断：按钮按下 → 增加频率档位
// ================================================================
void changeFrequencyISR() {
  unsigned long now = millis();
  if (now - lastButtonTime > 200) {
    freqLevel++;
    if (freqLevel > 10) freqLevel = 1;
    needsUpdate = true;
    lastButtonTime = now;
  }
}

// ================================================================
//        检查 SPDT 开关变化
// ================================================================
void checkModeChange() {
  int switchState = digitalRead(SWITCH_PIN);
  if (switchState != lastSwitchState) {
    lastSwitchState = switchState;
    freqLevel = 1; // 切换模式时重置到1Hz档
    needsUpdate = true;
  }
}

// ================================================================
//        LED 闪烁 (非阻塞)
// ================================================================
void updateLED() {
  int switchState = digitalRead(SWITCH_PIN);
  float actualFreq = (switchState == HIGH) ? freqLevel * 10.0 : freqLevel;
  long halfPeriod = (long)(500.0 / actualFreq);
  unsigned long now = millis();
  if (now - previousMillis >= halfPeriod) {
    previousMillis = now;
    ledState = (ledState == LOW) ? HIGH : LOW;
    digitalWrite(LED_PIN, ledState);
  }
}

// ================================================================
//        串口打印状态信息
// ================================================================
void printStatus() {
  int switchState = digitalRead(SWITCH_PIN);
  float actualFreq = (switchState == HIGH) ? freqLevel * 10.0 : freqLevel;

  Serial.println("-----------------------");
  Serial.print("Switch mode: ");
  Serial.println((switchState == HIGH) ? "x10 MODE" : "NORMAL MODE");
  Serial.print("Freq Level (1-10): ");
  Serial.println(freqLevel);
  Serial.print("Actual Frequency: ");
  Serial.print(actualFreq);
  Serial.println(" Hz");
}
