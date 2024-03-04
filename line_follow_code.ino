#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint8_t enA = 5;
uint8_t RightMotorA = 6;
uint8_t RightMotorB = 7;
uint8_t LeftMotorA = 9;
uint8_t LeftMotorB = 8;
uint8_t enB = 10;


int lastError = 0;
long integral = 0;


float Kp = 0.3; 
float Ki = 0.2; 
float Kd = 0.5; 

void setup() {
  pinMode(RightMotorA, OUTPUT);
  pinMode(RightMotorB, OUTPUT);
  pinMode(LeftMotorA, OUTPUT);
  pinMode(LeftMotorB, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(2);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void MoveRightFront(uint16_t velocity) {
  digitalWrite(RightMotorA, HIGH);
  digitalWrite(RightMotorB, LOW);
  analogWrite(enA, velocity);
}

void MoveLeftFront(uint16_t velocity) {
  digitalWrite(LeftMotorA, HIGH);
  digitalWrite(LeftMotorB, LOW);
  analogWrite(enB, velocity);
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  Serial.println(position);

  // PID computation
  int error = 3500 - position; // Assuming 3500 is the desired center position
  integral += error;
  int derivative = error - lastError;

  int motorspeed = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

 
  int motorspeedRight = constrain(255 + motorspeed, 0, 255); 
  int motorspeedLeft = constrain(255 - motorspeed, 0, 255);

  MoveRightFront(motorspeedRight);
  MoveLeftFront(motorspeedLeft);

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
}
