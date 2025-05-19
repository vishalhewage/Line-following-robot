// IR sensor pins
const int leftSensorPin = 34;   // GPIO34 (analog-capable input-only pin)
const int rightSensorPin = 35;  // GPIO35 (analog-capable input-only pin)

// Motor control pins
const int leftMotor1 = 18;
const int leftMotor2 = 19;
const int rightMotor1 = 23;
const int rightMotor2 = 22;

void setup() {
  Serial.begin(115200);

  // Set sensor pins as input
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  // Set motor pins as output
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
}

void loop() {
  int leftSensor = digitalRead(leftSensorPin);
  int rightSensor = digitalRead(rightSensorPin);

  Serial.print("Left: ");
  Serial.print(leftSensor);
  Serial.print(" | Right: ");
  Serial.println(rightSensor);

  if (leftSensor == LOW && rightSensor == LOW) {
    // Both sensors on the line → Go forward
    moveForward();
  }
  else if (leftSensor == LOW && rightSensor == HIGH) {
    // Left on line, right off → Turn left
    turnLeft();
  }
  else if (leftSensor == HIGH && rightSensor == LOW) {
    // Right on line, left off → Turn right
    turnRight();
  }
  else {
    // Both sensors off line → Stop or Search
    stopMotors();
  }

  delay(10);
}

// Motor control functions
void moveForward() {
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
}

void turnLeft() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
}

void turnRight() {
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
}

void stopMotors() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
}

