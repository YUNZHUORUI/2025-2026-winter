// Pin assignments
int greenPin = 4;
int yellowPin = 7;
int redPin = 10;

void setup() {
  // Set the pins as outputs
  pinMode(greenPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(redPin, OUTPUT);
}

void loop() {
  // Green light ON for 5 seconds
  digitalWrite(greenPin, HIGH);
  digitalWrite(yellowPin, LOW);
  digitalWrite(redPin, LOW);
  delay(5000);

  // Yellow light ON for 3 seconds
  digitalWrite(greenPin, LOW);
  digitalWrite(yellowPin, HIGH);
  digitalWrite(redPin, LOW);
  delay(3000);

  // Red light ON for 8 seconds
  digitalWrite(greenPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(redPin, HIGH);
  delay(8000);
}
