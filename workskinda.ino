#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>

// Sensor and Motor Definitions
MeLightSensor lightSensorFront(12);
MeEncoderOnBoard motorLeft(SLOT1);
MeEncoderOnBoard motorRight(SLOT2);
MeUltrasonicSensor ultrasonicFront(10);
MeRGBLed rgbLedMain(0, 12);
MeUltrasonicSensor ultrasonicLeft(6);
MeUltrasonicSensor ultrasonicRight(7);



double leftDist = 0;
double rightDist = 0;
double frontDist = 0;
double fullSpeed = 30; // in %
float rotationDelay = 0.8;
float motoradjust = 0;  // Adjust this value as needed


void isr_process_motorLeft(void)
{
  if(digitalRead(motorLeft.getPortB()) == 0) {
    motorLeft.pulsePosMinus();
  } else {
    motorLeft.pulsePosPlus();
  }
}

void isr_process_motorRight(void)
{
  if(digitalRead(motorRight.getPortB()) == 0) {
    motorRight.pulsePosMinus();
  } else {
    motorRight.pulsePosPlus();
  }
}

void move(int direction, int speed) {
  int leftSpeed = 0;
  int rightSpeed = 0;
  
  if(direction == 1) { // Forward
    leftSpeed = -speed - motoradjust; // Adjust left motor speed
    rightSpeed = speed;
  } else if(direction == 2) { // Backward
    leftSpeed = speed + motoradjust;  // Adjust left motor speed
    rightSpeed = -speed;
  } else if(direction == 3) { // Turn Left
    leftSpeed = -speed - motoradjust; // Adjust left motor speed
    rightSpeed = -speed;
  } else if(direction == 4) { // Turn Right
    leftSpeed = speed + motoradjust;  // Adjust left motor speed
    rightSpeed = speed;
  }

  motorLeft.setTarPWM(leftSpeed);
  motorRight.setTarPWM(rightSpeed);
}


void _delay(float seconds) {
  if(seconds < 0.0) {
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void setup() {
  randomSeed((unsigned long)(lightSensorFront.read() * 123456));
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(motorRight.getIntNum(), isr_process_motorRight, RISING);
  attachInterrupt(motorLeft.getIntNum(), isr_process_motorLeft, RISING);
  rgbLedMain.setpin(44);
  rgbLedMain.fillPixelsBak(0, 2, 1);
/*
  motorLeft.setPulse(9);
  motorLeft.setRatio(39.267);
  motorLeft.setPosPid(1.8,0,1.2);
  motorLeft.setSpeedPid(0.18,0,0);

  motorRight.setPulse(9);
  motorRight.setRatio(39.267);
  motorRight.setPosPid(1.8,0,1.2);
  motorRight.setSpeedPid(0.18,0,0);*/
}

void _loop() {
  motorRight.loop();
  motorLeft.loop();
}

unsigned long lastMovementTime = millis();
int previousPositionLeft = motorLeft.getCurPos();
int previousPositionRight = motorRight.getCurPos();

void checkIfStuck() {
  int currentPositionLeft = motorLeft.getCurPos();
  int currentPositionRight = motorRight.getCurPos();
  
  // If no change in encoder position for 2 seconds, assume stuck
  if (abs(currentPositionLeft - previousPositionLeft) < 3 || 
      abs(currentPositionRight - previousPositionRight) < 3) {
    if (millis() - lastMovementTime > 3000) {  // 2 seconds
      // The robot is stuck
      rgbLedMain.setColor(255, 255, 255, 144);
      rgbLedMain.show();
      if(ultrasonicRight.distanceCm()<ultrasonicLeft.distanceCm()){
        motorLeft.setTarPWM( 10 / 100.0 * 255);
        motorRight.setTarPWM(-15 / 100.0 * 255);
      }else {
        motorLeft.setTarPWM( 15 / 100.0 * 255);
        motorRight.setTarPWM(-10 / 100.0 * 255);
        
      }
     
     
      _delay(1.5);

      // Recovery is done, now reset last movement time
      lastMovementTime = millis();
    }
  } else {
    lastMovementTime = millis();  // Reset timer if movement is detected
  }
  // Update previous positions
  previousPositionLeft = currentPositionLeft;
  previousPositionRight = currentPositionRight;
}


void loop() {
    rgbLedMain.setColor(0, 0, 0, 0);
    //Motor right and left are swapped
    if(ultrasonicFront.distanceCm() > 30) {
      move(1, 30 / 100.0 * 255);
    }else if(ultrasonicFront.distanceCm() > 20) {
      move(1, 15 / 100.0 * 255);
    };

    if(ultrasonicFront.distanceCm() < 13)
    {
      move(1, 0);

      rgbLedMain.setColor(0, 255, 0, 144);
      rgbLedMain.show();
      _delay(1);

      // Check if left is free to move
      if (ultrasonicRight.distanceCm() > 16) {
        motorRight.setTarPWM(20 / 100.0 * 255);
        motorLeft.setTarPWM(0);
        _delay(1);
      }else  if(ultrasonicLeft.distanceCm() > 16) {
        motorRight.setTarPWM(0);
        motorLeft.setTarPWM(-1 * 20 / 100.0 * 255);
        _delay(1);
      } else {
        move(2, 10 / 100.0 * 255);
        _delay(1);
      }
    }
    /*
    // Right Obstacle
    if(ultrasonicRight.distanceCm() < 5) {
      motorRight.setTarPWM(-12/ 100.0 * 255);
      motorLeft.setTarPWM(10/ 100.0 * 255);
      rgbLedMain.setColor(0, 255, 0, 0);
      rgbLedMain.show();

    }
      
    // left Obstacle
    if(ultrasonicLeft.distanceCm() < 5) {
      motorLeft.setTarPWM(12/ 100.0 * 255);
      motorRight.setTarPWM(-10/ 100.0 * 255);
      rgbLedMain.setColor(0, 84, 255, 0);
      rgbLedMain.show();
    }*/

    checkIfStuck();

    _loop();
}
