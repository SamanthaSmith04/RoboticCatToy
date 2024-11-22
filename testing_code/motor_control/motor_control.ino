#include <Adafruit_NeoPixel.h> // Include the Adafruit NeoPixel library

int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 14;

int motor2Pin1 = 25;
int motor2Pin2 = 19;
int enable2Pin = 32;

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

void setup() {
  Serial.begin(9600);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);


  ledcSetup(motor1Pin1, freq, resolution);
  ledcSetup(motor1Pin2, freq, resolution);

}

void setMotor(int pin1, int pin2, int enablePin, int dutyCycle, int dir) {
  if (dir == 0) { // Stop
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    analogWrite(enablePin, 0); // Stop the motor
  } else if (dir == 1) { // Move forward
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(enablePin, dutyCycle); // Set speed
  } else { // Move backward
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(enablePin, dutyCycle); // Set speed
  }
}


void loop() {
  // Move the DC motor forward at maximum speed
  Serial.println("Moving Forward");
  setMotor(motor1Pin1, motor1Pin2, enable1Pin, 200, 1);
  delay(2000);
  Serial.println("stopping");
  setMotor(motor1Pin1, motor1Pin2, enable1Pin, 0, 0);
  delay(2000);
}