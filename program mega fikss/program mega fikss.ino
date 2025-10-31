//library 
#include <Servo.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "HX711.h"

// Konfigurasi Servo
Servo myServo;
int potPin = A0;
int potValue = 0;
int servoAngle = 90;

// Konfigurasi DHT22
#define dht_pin 35
#define dht_type DHT22
DHT dht22(dht_pin, dht_type);

// Konfigurasi HCSR04
#define triq1 8
#define echo1 34
#define triq3 2
#define echo3 30
#define triq2 3
#define echo2 32

// Konfigurasi Sensor Getaran
#define VIBRATION_PIN 44
unsigned long lastVibrationTime = 0;
unsigned long vibrationInterval = 0;
unsigned long noVibrationTime = 0;
// int vibrationCount = 0;
float vibrationFrequency = 0;

// Konfigurasi HX711
#define DT 46
#define SCK 48
HX711 scale;
float calibration_factor = -81.0;

// Konfigurasi LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Konfigurasi button 
const int button1 = 39; //motor 1 kri
const int button2 = 47; //motor 1 kanan
const int button3 = 41; //motor 2 naik
const int button4 = 49; //motor 2 turun
const int buttonForward = 37; //maju
const int buttonBackward = 43; //mundur
//motor kri kanan (X)
const int motor1_in1 = 23; 
const int motor1_in2 = 25;
const int motor1_ena = 11;
//motor atas bawah (Y)
const int motor2_in3 = 27;
const int motor2_in4 = 29;
const int motor2_enb = 10;
//motor maju mundur(z)
const int motor_z_in1 = 22; 
const int motor_z_in2 = 24;
const int motor_z_ena = 12;

int x = 0; int y = 0; int z = 0;

// Timer untuk interval pembacaan sensor
unsigned long lastSensorReadTime = 0;
const unsigned long sensorInterval = 500; // Interval pembacaan sensor dalam ms

// Data sensor
float humi, temp, weight;
long jarakX, jarakY, jarakZ;

void setup() {
  Serial1.begin(115200); // Serial untuk komunikasi ke ESP8266
  Serial.begin(9600);  // Serial untuk debugging

  dht22.begin();

  pinMode(triq1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(triq2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(triq3, OUTPUT);
  pinMode(echo3, INPUT);
  pinMode(VIBRATION_PIN, INPUT_PULLUP);

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  pinMode(buttonForward, INPUT_PULLUP);
  pinMode(buttonBackward, INPUT_PULLUP);

  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor1_ena, OUTPUT);

  pinMode(motor2_in3, OUTPUT);
  pinMode(motor2_in4, OUTPUT);
  pinMode(motor2_enb, OUTPUT);

  pinMode(motor_z_in1, OUTPUT);
  pinMode(motor_z_in2, OUTPUT);
  pinMode(motor_z_ena, OUTPUT);

  myServo.attach(5);

  lcd.init();
  lcd.backlight();

  scale.begin(DT, SCK);
  scale.set_scale(calibration_factor);
  scale.tare();
}

void loop() {
  unsigned long currentMillis = millis();

  // Membaca sensor setiap interval
  if (currentMillis - lastSensorReadTime >= sensorInterval) {
    lastSensorReadTime = currentMillis;

    humi = dht22.readHumidity();
    temp = dht22.readTemperature();
    jarakX = readUltrasonic(triq1, echo1);
    jarakY = readUltrasonic(triq3, echo3);
    jarakZ = readUltrasonic(triq2, echo2);
    weight = scale.get_units();
    weight = (abs(weight) < 1.0) ? 0 : weight;

    updateVibration();

    // Menampilkan data di LCD
    lcd.setCursor(0, 0);
    lcd.print("H:" + String(humi, 1) + "% T:" + String(temp, 1) + "C");
    lcd.setCursor(0, 1);
    lcd.print("Berat: " + String(weight, 1) + " g");
    lcd.setCursor(0, 2);
    lcd.print("X:" + String(jarakX) + " Y:" + String(jarakY) + " Z:" + String(jarakZ));
    lcd.setCursor(0, 3);
    lcd.print("Vib:" + String(vibrationFrequency, 1));
    
    // delay(200);
    // Mengirim data ke ESP8266 berulang
    sendDataToESP8266();

    // Debugging data pada Serial Monitor
    Serial.println("=== Debugging Data ===");
    Serial.print("Humidity: "); Serial.print(humi); Serial.println("%");
    Serial.print("Temperature: "); Serial.print(temp); Serial.println(" C");
    Serial.print("Distance X: "); Serial.print(jarakX); Serial.println(" cm");
    Serial.print("Distance Y: "); Serial.print(jarakY); Serial.println(" cm");
    Serial.print("Distance Z: "); Serial.print(jarakZ); Serial.println(" cm");
    Serial.print("Weight: "); Serial.print(weight); Serial.println(" g");
    Serial.print("Vibration Frequency: "); Serial.print(vibrationFrequency); Serial.println(" Hz");
    // Serial.print("Vibration Count: "); Serial.println(vibrationCount);
    Serial.println("=====================");
  }

  // Mengatur servo
  potValue = analogRead(potPin);
  int mappedValue = map(potValue, 0, 1023, -90, 90);
  if (mappedValue > -10 && mappedValue < 10) {
    myServo.write(servoAngle);
  } else {
    servoAngle = constrain(servoAngle + mappedValue / 50, 0, 180);
    myServo.write(servoAngle);
  }

  // Membaca tombol dan mengontrol motor
  bool button1State = !digitalRead(button1);
  bool button2State = !digitalRead(button2);
  bool button3State = !digitalRead(button3);
  bool button4State = !digitalRead(button4);
  bool buttonForwardState = !digitalRead(buttonForward);
  bool buttonBackwardState = !digitalRead(buttonBackward);

  x = button1State ? -1 : button2State ? 1 : 0;
  y = button3State ? 1 : button4State ? -1 : 0;
  z = buttonForwardState ? 1 : buttonBackwardState ? -1 : 0;

  controlMotor1(x); controlMotor2(y); controlMotorZ(z);
}
// kirim data ke 8266
void sendDataToESP8266() {
  String jsonData = "{";
  jsonData += "\"humidity\":" + String(humi, 1) + ",";
  jsonData += "\"temperature\":" + String(temp, 1) + ",";
  jsonData += "\"distanceX\":" + String(jarakX) + ",";
  jsonData += "\"distanceY\":" + String(jarakY) + ",";
  jsonData += "\"distanceZ\":" + String(jarakZ) + ",";
  jsonData += "\"weight\":" + String(weight, 1) + ",";
  jsonData += "\"vibrationFrequency\":" + String(vibrationFrequency, 1) + ",";
  // jsonData += "\"vibrationCount\":" + String(vibrationCount);
  jsonData += "}";

  Serial1.println(jsonData);
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return (duration > 0) ? (duration / 2) / 29.1 : -1;
}

void updateVibration() {
  int vibrationValue = digitalRead(VIBRATION_PIN);
  unsigned long currentTime = micros();

  if (vibrationValue == HIGH) {
    if (currentTime - lastVibrationTime > 30) {//debounce time
      // vibrationCount++;
      vibrationInterval = currentTime - lastVibrationTime;
      lastVibrationTime = currentTime;
    }
    noVibrationTime = millis();
  } else {
    if (millis() - noVibrationTime > 2000) {
      vibrationInterval = 0;
      vibrationFrequency = 0;
      // vibrationCount = 0;
    }
  }

  if (vibrationInterval > 0) {
    vibrationFrequency = 1000000.0 / vibrationInterval;
  }
}
//motor kanan kiri
void controlMotor1(int state) {
  if (state == -1) {
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);
    analogWrite(motor1_ena, 100);//speed
  } else if (state == 1) {
    digitalWrite(motor1_in1, LOW);
    digitalWrite(motor1_in2, HIGH);
    analogWrite(motor1_ena, 100);//speed 
  } else {
    digitalWrite(motor1_in1, LOW);
    digitalWrite(motor1_in2, LOW);
  }
}
//motor naik turun
void controlMotor2(int state) {
  if (state == 1) {
    digitalWrite(motor2_in3, HIGH);
    digitalWrite(motor2_in4, LOW);
    analogWrite(motor2_enb, 80);//speed naik
  } else if (state == -1) {
    digitalWrite(motor2_in3, LOW);
    digitalWrite(motor2_in4, HIGH);
    analogWrite(motor2_enb, 50);//speed turun
  } else {
    digitalWrite(motor2_in3, LOW);
    digitalWrite(motor2_in4, LOW);
  }
}
//motor maju mundur
void controlMotorZ(int state) {
  if (state == 1) {
    digitalWrite(motor_z_in1, HIGH);
    digitalWrite(motor_z_in2, LOW);
    analogWrite(motor_z_ena, 200);//speed maju
  } else if (state == -1) {
    digitalWrite(motor_z_in1, LOW);
    digitalWrite(motor_z_in2, HIGH);
    analogWrite(motor_z_ena, 200);//speed mundur
  } else {
    digitalWrite(motor_z_in1, LOW);
    digitalWrite(motor_z_in2, LOW);
  }
}
