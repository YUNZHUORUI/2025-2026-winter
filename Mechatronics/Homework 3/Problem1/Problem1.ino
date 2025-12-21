

int pinA = 2;
int pinB = 3;
int pinC = 4;
int pinD = 5;
int pinE = 6;
int pinF = 7;
int pinG = 8;
int pinDP = 13;  // 小数点引脚（新增！）

int D1 = 9;   // 千位
int D2 = 10;  // 百位
int D3 = 11;  // 十位
int D4 = 12;  // 个位

int displayPins[4] = {D1, D2, D3, D4};

// 共阳段码表：{A,B,C,D,E,F,G,DP}，0=亮(LOW)，1=灭(HIGH)
byte digits[10][8] = {
  {0,0,0,0,0,0,1,1},  // 0
  {1,0,0,1,1,1,1,1},  // 1
  {0,0,1,0,0,1,0,1},  // 2
  {0,0,0,0,1,1,0,1},  // 3
  {1,0,0,1,1,0,0,1},  // 4
  {0,1,0,0,1,0,0,1},  // 5
  {0,1,0,0,0,0,0,1},  // 6
  {0,0,0,1,1,1,1,1},  // 7
  {0,0,0,0,0,0,0,1},  // 8
  {0,0,0,0,1,0,0,1}   // 9
};

void setup() {
  for (int i = 2; i <= 13; i++) {
    pinMode(i, OUTPUT);
  }
  // 可选：串口调试
  // Serial.begin(9600);
}

void loop() {
  // 读取电压
  int raw = analogRead(A0);
  float voltage = raw * 5.0 / 1023.0;  // 0.000 ~ 5.000

  // 转换为 4 位数字 + 小数点位置
  int thousands = (int)voltage;                    // 整数部分
  int hundreds  = (int)(voltage * 10)    % 10;     // 小数第1位
  int tens      = (int)(voltage * 100)   % 10;     // 小数第2位
  int ones      = (int)(voltage * 1000)  % 10;     // 小数第3位

  int digitValues[4] = {thousands, hundreds, tens, ones};
  bool decimalPoints[4] = {true, false, false, false};  // 只在第2位后亮小数点

  // 动态显示
  displayFourDigitsWithDP(digitValues, decimalPoints);
}

// 动态扫描显示4位数字 + 控制小数点
void displayFourDigitsWithDP(int values[4], bool dp[4]) {
  for (int pos = 0; pos < 4; pos++) {
    clearAllSegments();
    digitalWrite(displayPins[pos], HIGH);  // 选通当前位

    int num = values[pos];
    if (num >= 0 && num <= 9) {
      digitalWrite(pinA,  digits[num][0]);
      digitalWrite(pinB,  digits[num][1]);
      digitalWrite(pinC,  digits[num][2]);
      digitalWrite(pinD,  digits[num][3]);
      digitalWrite(pinE,  digits[num][4]);
      digitalWrite(pinF,  digits[num][5]);
      digitalWrite(pinG,  digits[num][6]);
      digitalWrite(pinDP, dp[pos] ? LOW : HIGH);  // 小数点：true=亮
    }
    delay(1);  // 扫描速度 ~125Hz，无闪烁
  }
}

void clearAllSegments() {
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
  digitalWrite(pinDP, HIGH);
  // 关闭其他位
  for (int i = 0; i < 4; i++) {
    digitalWrite(displayPins[i], LOW);
  }
}