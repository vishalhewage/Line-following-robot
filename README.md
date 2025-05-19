#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// === Pins ===
const int trigPin = 8;
const int echoPin = 9;
const int ldrPin = A0;
const int waterSensorPin = 6;
const int panicButtonPin = 2;
const int buzzerPin = 5;

// === Serial Interfaces ===
SoftwareSerial gpsSerial(4, 3);    // GPS: RX, TX
SoftwareSerial gsmSerial(10, 11);  // GSM: RX, TX
TinyGPSPlus gps;

// === Button Logic Variables ===
unsigned long lastPressTime = 0;
int pressCount = 0;
bool smsSent = false;
bool buzzerActive = false;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(waterSensorPin, INPUT);
  pinMode(panicButtonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  // Initialize GSM
  delay(10000); // allow GSM to register on network
  gsmSerial.println("AT");
  delay(500);
  gsmSerial.println("AT+CMGF=1");
  delay(500);
  gsmSerial.println("AT+CNMI=1,2,0,0,0");
  delay(500);
}

void loop() {
  // === Sensor Checks ===
  long duration;
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;

  int lightValue = analogRead(ldrPin);
  int waterStatus = digitalRead(waterSensorPin);
  bool panicPressed = digitalRead(panicButtonPin) == LOW;

  // === Buzzer Logic ===
  if ((distance < 50 || lightValue < 200 || waterStatus == LOW) && !panicPressed) {
    digitalWrite(buzzerPin, HIGH);
    buzzerActive = true;
  }

  // Stop buzzer if long press (held for > 2 sec)
  if (panicPressed) {
    static unsigned long buttonPressStart = 0;
    if (buttonPressStart == 0) {
      buttonPressStart = millis();
    } else if (millis() - buttonPressStart >= 2000) {
      digitalWrite(buzzerPin, LOW);
      buzzerActive = false;
    }
  } else {
    buttonPressStart = 0; // reset if button released
    if (!buzzerActive) digitalWrite(buzzerPin, LOW);
  }

  // === Double Press Logic for SMS ===
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(panicButtonPin);

  if (lastButtonState == HIGH && buttonState == LOW) {
    unsigned long currentTime = millis();
    if (currentTime - lastPressTime <= 1000) {
      pressCount++;
    } else {
      pressCount = 1;
    }
    lastPressTime = currentTime;
  }
  lastButtonState = buttonState;

  if (pressCount >= 2 && !smsSent) {
    sendEmergencySMS();
    smsSent = true;
  }

  // Reset after release
  if (digitalRead(panicButtonPin) == HIGH && smsSent) {
    delay(1000);
    pressCount = 0;
    smsSent = false;
  }

  // Update GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

void sendEmergencySMS() {
  String lat = gps.location.isValid() ? String(gps.location.lat(), 6) : "0.000000";
  String lon = gps.location.isValid() ? String(gps.location.lng(), 6) : "0.000000";
  String message = "Emergency! Help needed.\nLocation: https://maps.google.com/?q=" + lat + "," + lon;

  gsmSerial.println("AT+CMGF=1");
  delay(1000);
  gsmSerial.println("AT+CMGS=\"+94713313777\"");
  delay(1000);
  gsmSerial.print(message);
  gsmSerial.write(26); // Ctrl+Z
  delay(5000);
}
