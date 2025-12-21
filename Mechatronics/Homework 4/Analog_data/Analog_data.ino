int potpin=0;//定义模拟接口0

int ledpin=13;//定义数字接口13

int val=0;//将定义变量val,并赋初值0

void setup()

{

pinMode(ledpin,OUTPUT);//定义数字接口为输出接口

Serial.begin(9600);//设置波特率为9600

}

void loop()

{

digitalWrite(ledpin,HIGH);//点亮数字接口13 的LED

delay(50);//延时0.05 秒

digitalWrite(ledpin,LOW);//熄灭数字接口13 的LED

delay(50);//延时0.05 秒

val=analogRead(potpin);//读取模拟接口0 的值，并将其赋给val

Serial.println(val);//显示出val 的值

}