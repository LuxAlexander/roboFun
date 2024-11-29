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
MeBuzzer buzzer;



double leftDist = 0;
double rightDist = 0;
double frontDist = 0;
double fullSpeed = 30; // in %
float rotationDelay = 0.8;
float motoradjust = 0;  // Adjust this value as needed
int currSpeed = 0;
float oldDistance = 0;
unsigned int curveCorrectionLeft = 0;
unsigned int curveCorrectionRight = 0;

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
  Serial.begin(115200);
  buzzer.setpin(45);
  randomSeed((unsigned long)(lightSensorFront.read() * 123456));
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(motorRight.getIntNum(), isr_process_motorRight, RISING);
  attachInterrupt(motorLeft.getIntNum(), isr_process_motorLeft, RISING);
  rgbLedMain.setpin(44);
  rgbLedMain.fillPixelsBak(0, 2, 1);

  buzzer.tone(330, 0.1 * 1000);  // E4 (high beep)
  delay(100);                    // Small delay
  buzzer.tone(220, 0.2 * 1000);  // A3 (lower boop)
  delay(150);                    // Small delay
  buzzer.tone(330, 0.1 * 1000);  // E4 (high beep again)
  delay(100);                    // Small delay
  buzzer.tone(174, 0.15 * 1000); // D3 (lower bup)
  delay(200);     

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
        _delay(2);
      }else {
        motorLeft.setTarPWM( 15 / 100.0 * 255);
        motorRight.setTarPWM(-10 / 100.0 * 255);
        _delay(2);
      }
     
      //1.5 not enoug, 2 is good 2.5 too long
      

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
    rgbLedMain.show();

    if(ultrasonicFront.distanceCm() > 30) {
      move(1, 30 / 100.0 * 255);
      currSpeed = 30 / 100.0 * 255;
    }else if(ultrasonicFront.distanceCm() > 20) {
      move(1, 15 / 100.0 * 255);
      currSpeed = 15 / 100.0 * 255;
    }


    if(ultrasonicFront.distanceCm() < 13)
    {
      move(1, 0);

      curveCorrectionLeft = 420;
      curveCorrectionRight = 420;

      rgbLedMain.setColor(0, 255, 0, 144);
      rgbLedMain.show();
      _delay(1);

      // Check if left is free to move
      //1.3 too much
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

    checkIfStuck();

    if (ultrasonicFront.distanceCm() != 400 || ultrasonicFront.distanceCm() < 300) {
      oldDistance = ultrasonicFront.distanceCm();
    }

    if (oldDistance > 25)
    {
      Serial.println(oldDistance);
      wallCorrection();
    }
    
    _loop();
}

void wallCorrection() {
   // Wall Correction

    if (curveCorrectionLeft == 420 && curveCorrectionLeft == 420) {
      curveCorrectionLeft = 0;
      curveCorrectionRight = 0;
      return;
    }
    

    // Rechts
    if(ultrasonicRight.measure() < 430 && curveCorrectionRight < 150) {
      motorRight.setTarPWM(currSpeed * 1);
      motorLeft.setTarPWM(currSpeed * (-1.2));
      _delay(0.001);
      curveCorrectionRight++;
      

      rgbLedMain.setColor(0, 84, 255, 0);
      rgbLedMain.show();

      curveCorrectionLeft = 0;

      return;
    }

    // Links
    if(ultrasonicLeft.measure() < 430 && curveCorrectionLeft < 150) {
      motorRight.setTarPWM(currSpeed * 1.2);
      motorLeft.setTarPWM(currSpeed * (-1));
      _delay(0.001);
      curveCorrectionLeft++;
      

      rgbLedMain.setColor(0, 255, 0, 0);
      rgbLedMain.show();
      curveCorrectionRight = 0;

      return;
    }
}