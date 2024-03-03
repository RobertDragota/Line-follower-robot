#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

float Kp = 0; 
float Ki = 0; 
float Kd = 0; 
int P, I, D;

int lastError = 0;
boolean onoff = false;

const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

int mode = 8;
int aphase = 9;
int aenbl = 6;
int bphase = 5;
int benbl = 3;

int buttoncalibrate = 17; 
int buttonstart = 2;

void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){10, 11, 12, 14, 15, 16, 18, 19}, SensorCount);
  qtr.setEmitterPin(7);

  pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  digitalWrite(mode, HIGH); 
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  while (!digitalRead(buttoncalibrate)) {
    delay(10); // Wait for the button to be pressed
  }
  calibration(); 
  forward_brake(0, 0); 
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if(digitalRead(buttonstart) == HIGH) {
    onoff = !onoff;
    delay(onoff ? 1000 : 50);
  }
  if (onoff) {
    PID_control();
  } else {
    forward_brake(0,0); 
  }
}

void forward_brake(int posa, int posb) {
  digitalWrite(aphase, LOW);
  digitalWrite(bphase, LOW);
  analogWrite(aenbl, posa);
  analogWrite(benbl, posb);
}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues); 
  int error = 3500 - position; 

  P = error;
  I += error;
  D = error - lastError;

  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; 

  int motorspeeda = constrain(basespeeda + motorspeed, 0, maxspeeda);
  int motorspeedb = constrain(basespeedb - motorspeed, 0, maxspeedb);
  
  forward_brake(motorspeeda, motorspeedb);
}
