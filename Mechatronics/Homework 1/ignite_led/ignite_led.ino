int ledpin = 10;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(ledpin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(ledpin,HIGH);
delay(2);//3, 5, 7.5, 9
digitalWrite(ledpin,LOW);
delay(8);//7, 5, 2.5, 1
}
