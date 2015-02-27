/*
  Group 14 Arduino Code
*/

#include <Wire.h>
#include <Servo.h>

#include "SDPArduino.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>


byte cmd, data, response;
Servo kicker;
unsigned long current_millis;

// For timing purposes
unsigned long current_micros;
unsigned long old_micros;
unsigned long old_millis;

byte max_time = 0;    // the maximum time (in ms)
byte min_time = 255;  // the minimum time (in us)

int targetHeading, headingDiff;

int turnPower;

/* Assign a unique ID to the sensors */

Adafruit_9DOF                  dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified  accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified    mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified        gyro = Adafruit_L3GD20_Unified(20);

typedef struct Command {
  unsigned long millis;
  byte data;
  void (*functionPtr)(byte);
} Command;

Command commands[7];

/*
  Command class bytes. Set to kgaming-related stuff
  now for debugging purposes, but will change to something
  more professional-looking in the future.
*/

#define _FORWARD 'W'
#define _BACKWARD 'S' 
#define _STOP ' '
#define _KICK 'Q'
#define _HEARTBEAT 'L'
#define _GRABBER_OPEN 'Z'
#define _GRABBER_CLOSE 'X'
#define _GET_HEADING 'U'
#define _TURN_LEFT 'A'
#define _TURN_RIGHT 'D'
#define _STRAFE_LEFT 'C'
#define _STRAFE_RIGHT 'V'
#define _GET_TIMING 'T'

#define _LEFT_DRIVE 2
#define _RIGHT_DRIVE 4
#define _BACK_DRIVE 3
#define _LEFT_GRABBER 1
#define _RIGHT_GRABBER 5

#define _HEADING_TOLERANCE 1
#define _HEADING_CORRECTION_COEFFICIENT 0.5
#define _TURN_DELAY 50

#define _RAD_TO_DEG 57.2957795


/*
  Setup function. Performs the standard SDPsetup routine supplied by
  the course staff. Also sets up the kicker servo.
*/

void setup() {
  SDPsetup();
  // attach the kicker servo to pin 5, with 600us to 2400us pulses.
  kicker.attach(5, 600, 2400); 
  // initialise the IMU
  initSensors();
  
  // initialise kick functions as they're constant
  commands[1].functionPtr = &doKick;
  commands[2].functionPtr = &lowerKicker;
  commands[3].functionPtr = &resetKicker;
}

void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
  }
  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  
  /* Initialise the sensor */
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
  }
}

/*
  Main loop. Runs an endless loop of fetching commands from the serial
  comms, decoding them, and generally doing the stuff we're supposed to do.
*/

void loop() {
  updateTimings();
  fetchCommand();
  if (cmd) {
    decodeCommand();
  }
  doStoredCommands();
  respond();
}

void updateTimings() {
  old_millis = current_millis;
  current_millis = millis();
  old_micros = current_micros;
  current_micros = micros();
  computeMinMaxTimings();
}

/*
  Fetches a single command (command byte + data byte) from
  the serial stream if there is data available.
*/

void fetchCommand() {
  cmd = 0x00;
  if (Serial.available() >= 2) {
    cmd = (char)Serial.read();
    data = (char)Serial.read();
    response = cmd;
  }
}

/*
  Decodes the current command byte, and executes the instruction given.
*/

void decodeCommand() {
  int deltaAngle;

  switch(cmd) {
    // movement commands
    case _FORWARD:
      if (commands[0].functionPtr != &moveForward) {
        targetHeading = getCurrentHeading();
      }
      motorStop(_BACK_DRIVE);
      commands[0].millis = current_millis;
      commands[0].functionPtr = &moveForward;
      commands[0].data = data; 
      break;

    case _BACKWARD:
      if (commands[0].functionPtr != &moveBackward) {
        targetHeading = getCurrentHeading();
      }
      motorStop(_BACK_DRIVE);
      commands[0].millis = current_millis;
      commands[0].functionPtr = &moveBackward;
      commands[0].data = data;
      break;

    case _STOP:
      commands[0].millis = current_millis;
      commands[0].functionPtr = &driveMotorStop;
      commands[0].data = 1;
      break;

    case _TURN_LEFT:
      motorStop(_BACK_DRIVE);
      turnPower = -30;
      commands[0].millis = current_millis;
      commands[0].functionPtr = &turnLeft;
      deltaAngle = (int)data * 2;
      targetHeading = (getCurrentHeading()  + deltaAngle) % 360;
      commands[0].data = getHeadingTolerance(getCurrentHeading(), targetHeading);
      break;

    case _TURN_RIGHT:
      motorStop(_BACK_DRIVE);
      turnPower = 30;
      commands[0].millis = current_millis;
      commands[0].functionPtr = &turnRight;
      deltaAngle = (int)data * 2;
      targetHeading = (getCurrentHeading()  - deltaAngle) % 360;
      commands[0].data = getHeadingTolerance(getCurrentHeading(), targetHeading);
      break;

    case _STRAFE_LEFT:
      if (commands[0].functionPtr != &strafe) {
        targetHeading = getCurrentHeading();
      }
      motorStop(_LEFT_DRIVE);
      motorStop(_RIGHT_DRIVE);
      motorStop(_BACK_DRIVE);
      commands[0].millis = current_millis + 200;
      commands[0].functionPtr = &startStrafingLeft;
      commands[0].data = data;
      break;

    case _STRAFE_RIGHT:
      if (commands[0].functionPtr != &strafe) {
        targetHeading = getCurrentHeading();
      }
      motorStop(_LEFT_DRIVE);
      motorStop(_RIGHT_DRIVE);
      motorStop(_BACK_DRIVE);
      commands[0].millis = current_millis + 200;
      commands[0].functionPtr = &startStrafingRight;
      commands[0].data = data;
      break;

    // kick commands
    case _KICK:
      if (data > 100) {
        data = 100;
      }
      kicker.write(90 - 4 * data / 100.0);
      commands[1].millis = current_millis + 400;
      commands[2].millis = current_millis + 800;
      commands[3].millis = current_millis + 1000;
      commands[1].data = commands[2].data = commands[3].data = data;
      break;

    // grabber commands
    case _GRABBER_OPEN:
      commands[4].millis = current_millis;
      commands[4].functionPtr = &startOpenGrabber;

      commands[5].millis = current_millis + 200;
      commands[5].functionPtr = &slowOpenGrabber;

      commands[6].millis = current_millis + 400;
      commands[6].functionPtr = &stopGrabber;

      commands[4].data = commands[5].data = commands[6].data = data;
      break;

    case _GRABBER_CLOSE:
      commands[4].millis = current_millis;
      commands[4].functionPtr = &startCloseGrabber;

      commands[5].millis = current_millis + 200;
      commands[5].functionPtr = &slowCloseGrabber;

      commands[6].millis = current_millis + 400;
      commands[6].functionPtr = &stopGrabber;

      commands[4].data = commands[5].data = commands[6].data = data;
      break;

    case _HEARTBEAT:
      heartbeat(data);
      break; 

    case _GET_HEADING:
      returnHeading();
      break;

    case _GET_TIMING:
      returnTiming(data);
      break;
     
   }
}

void doStoredCommands() {
  for (short i = 0; i < 7; i++) {
    if (commands[i].millis && commands[i].millis < current_millis) {
      if (commands[i].functionPtr != NULL) {
        (*commands[i].functionPtr)(commands[i].data);
      }
    }
  }
}

void respond() {
  if (response) {
    Serial.write(response);
  }
  response = 0x00;
}

void voidCommand(short i) {
  commands[i].millis = 0;
}

// Command functions
// move commands
void moveForward(byte power) {
  //check compass and do things
  int headingDiff = getHeadingDiff(targetHeading, getCurrentHeading());
  if (headingDiff > (_HEADING_TOLERANCE)) {
    // turn left....
    motorBackward(_LEFT_DRIVE, power - abs(headingDiff) * _HEADING_CORRECTION_COEFFICIENT);
    motorForward(_RIGHT_DRIVE, power);
  } else if (headingDiff < -(_HEADING_TOLERANCE)) {
    // turn right..
    motorBackward(_LEFT_DRIVE, power);
    motorForward(_RIGHT_DRIVE, power - abs(headingDiff) * _HEADING_CORRECTION_COEFFICIENT);
  } else {
    motorBackward(_LEFT_DRIVE, power);
    motorForward(_RIGHT_DRIVE, power);
  }
  commands[0].millis += 100;
}

void moveBackward(byte power) {
  //check compass and do things
  int headingDiff = getHeadingDiff(targetHeading, getCurrentHeading());
  if (headingDiff > (_HEADING_TOLERANCE)) {
    // turn left....
    motorForward(_LEFT_DRIVE, power - abs(headingDiff) * _HEADING_CORRECTION_COEFFICIENT);
    motorBackward(_RIGHT_DRIVE, power);
  } else if (headingDiff < -(_HEADING_TOLERANCE)) {
    // turn right..
    motorForward(_LEFT_DRIVE, power);
    motorBackward(_RIGHT_DRIVE, power - abs(headingDiff) * _HEADING_CORRECTION_COEFFICIENT);
  } else {
    motorForward(_LEFT_DRIVE, power); 
    motorBackward(_RIGHT_DRIVE, power);
  }
  commands[0].millis += 100;
}

void driveMotorStop(byte data) {
  motorBrake(_LEFT_DRIVE);
  motorBrake(_RIGHT_DRIVE);
  motorBrake(_BACK_DRIVE);
  
  commands[0].millis = millis() + 500;
  commands[0].functionPtr = &releaseBrakes;
  commands[0].data = 1;
}

void setRotationalSpeed(int target) {
  const int _THRESHOLD = 1;
  int currentGyro = getGyroOrientation();
  int gyroDiff = target - currentGyro; //negative if more ccw, positive if more cw.

  if (gyroDiff > _THRESHOLD) {
    turnPower+=1;
    if (turnPower > 100) {
      turnPower = 100;
    }
  }
  else if (gyroDiff < -(_THRESHOLD)) {
    turnPower-=1;
    if (turnPower < -100) {
      turnPower = -100;
    }
  }
 
  if (turnPower > 10) {
    motorBackward(_LEFT_DRIVE, turnPower);
    motorBackward(_RIGHT_DRIVE, turnPower);
  }
  else if (turnPower < -10) {
    motorForward(_LEFT_DRIVE, -turnPower);
    motorForward(_RIGHT_DRIVE, -turnPower);
  }
  else {
    motorStop(_LEFT_DRIVE);
    motorStop(_RIGHT_DRIVE);
  }
}

void turnLeft(byte tolerance) {
  // check compass and do things
  headingDiff = getHeadingDiff(targetHeading, getCurrentHeading());
  if (headingDiff < 0) {
    headingDiff = 360 + headingDiff;
  }
  if (headingDiff < tolerance) {
    // Place the motors in braking mode. (This is risky as f**k.)
    motorBrake(_LEFT_DRIVE);
    motorBrake(_RIGHT_DRIVE);
    motorBrake(_BACK_DRIVE);
    response = 0xff;

    // schedule the brakes to be released.
    commands[0].millis = millis() + 500;
    commands[0].functionPtr = &releaseBrakes;
    commands[0].data = 1;
  }
  else {    
    setRotationalSpeed(-getTurnSpeed(headingDiff));
    commands[0].millis = millis() + _TURN_DELAY;
  }
} 

void turnRight(byte tolerance) {
  //check compass and do things
  headingDiff = -getHeadingDiff(targetHeading, getCurrentHeading());
  if (headingDiff < 0) {
    headingDiff = 360 + headingDiff;
  }
  if (headingDiff < tolerance) {
     // Place the motors in braking mode. (This is risky as f**k.)
    motorBrake(_LEFT_DRIVE);
    motorBrake(_RIGHT_DRIVE);
    motorBrake(_BACK_DRIVE);
    response = 0xff;

    // schedule the brakes to be released.
    commands[0].millis = millis() + 500;
    commands[0].functionPtr = &releaseBrakes;
    commands[0].data = 1;
  }
  else {
    setRotationalSpeed(getTurnSpeed(headingDiff));
    commands[0].millis = millis() + _TURN_DELAY;
  }
}

void releaseBrakes(byte data) {
  if (data == 1) {
    motorStop(_LEFT_DRIVE);
    motorStop(_RIGHT_DRIVE);
    motorStop(_BACK_DRIVE);
    voidCommand(0);
  }
  else if (data == 2) {
    motorStop(_LEFT_GRABBER);
    motorStop(_RIGHT_GRABBER);
    voidCommand(6);
  }
}

void startStrafingRight(byte data) {
  commands[0].millis = current_millis;
  commands[0].functionPtr = &strafe;
  commands[0].data = data;
  // Try to anticipate the torque generated by the back wheel
  motorBackward(_LEFT_DRIVE, 40);
  motorBackward(_RIGHT_DRIVE, 40);
  // Then apply power to the back.
  motorForward(_BACK_DRIVE, data);
}

void startStrafingLeft(byte data) {
  commands[0].millis = current_millis;
  commands[0].functionPtr = &strafe;
  commands[0].data = data;
  // Try to anticipate the torque generated by the back wheel
  motorForward(_LEFT_DRIVE, 40);
  motorForward(_RIGHT_DRIVE, 40);
  // Then apply power to the back.
  motorBackward(_BACK_DRIVE, data);
}

void strafe(byte data) {
  int heading = getCurrentHeading();
  headingDiff = getHeadingDiff(targetHeading, heading);
  if (headingDiff > _HEADING_TOLERANCE) {
    motorForward(_LEFT_DRIVE, 40);
    motorForward(_RIGHT_DRIVE, 40);
  }
  else if (headingDiff < -(_HEADING_TOLERANCE)) {
    motorBackward(_LEFT_DRIVE, 40);
    motorBackward(_RIGHT_DRIVE, 40);
  }
  else {
    motorBrake(_LEFT_DRIVE);
    motorBrake(_RIGHT_DRIVE);
  }

  commands[0].millis += 50;
}

//grabber commands
void startOpenGrabber(byte data) {
  motorForward(_RIGHT_GRABBER, 100);
  motorBackward(_LEFT_GRABBER, 100);
  voidCommand(4);
}

void slowOpenGrabber(byte data) {
  motorForward(_RIGHT_GRABBER, 50);
  motorBackward(_LEFT_GRABBER, 50);
  voidCommand(5);
}

void startCloseGrabber(byte data) {
  motorForward(_LEFT_GRABBER, 100);
  motorBackward(_RIGHT_GRABBER, 100);
  voidCommand(4);
}

void slowCloseGrabber(byte data) {
  motorForward(_LEFT_GRABBER, 50);
  motorBackward(_RIGHT_GRABBER, 50);
  voidCommand(5);
}

void stopGrabber(byte data) {
  motorBrake(_LEFT_GRABBER);
  motorBrake(_RIGHT_GRABBER);

  // schedule the brakes to be released.
  commands[6].millis = millis() + 500;
  commands[6].functionPtr = &releaseBrakes;
  commands[6].data = 2;
}

//kicker commands

void doKick(byte power) {
  kicker.write(90 + 20 * power / 100.0);
  voidCommand(1);
}

void lowerKicker(byte power) {
  kicker.write(90 + 10 * power / 100.0);
  voidCommand(2);
}

void resetKicker(byte data) {
  kicker.write(90);
  voidCommand(3);
}

/*
  Sends a response to a received Hearbeat request by simply
  returning the request data. We could do something more sophisticated
  here, but for now it'll probably do.
*/

void heartbeat(byte _data) {
  response = _data;
}

void returnHeading() {
  response = (byte) (getCurrentHeading() / 2); 
}

void computeMinMaxTimings() {
  int timeDiffMicros = (current_micros - old_micros) / 10;
  if (timeDiffMicros > 255) {
    timeDiffMicros = 255;
  }
  if (timeDiffMicros == 0) {
    timeDiffMicros = 1;
  }
  int timeDiffMillis = (current_millis - old_millis) * 10;
  if (timeDiffMillis > 255) {
    timeDiffMillis = 255;
  }
  if (timeDiffMillis == 0) {
    timeDiffMillis = 1;
  }
  if (min_time > timeDiffMicros) min_time = (byte) timeDiffMicros;
  if (max_time < timeDiffMillis) max_time = (byte) timeDiffMillis;
}

void returnTiming(byte data) {
  if (!data) {
    int timeDiff = (current_micros - old_micros) / 10;
    if (timeDiff > 255) {
      timeDiff = 255;
    }
    if (timeDiff == 0) {
      timeDiff = 1;
    }
    response = (byte) timeDiff;
  }

  if (data == 1) {
    response = min_time;
  }
  else if (data == 2) {
    response = max_time;
  }
}

/*
  getHeadingDiff:

  Returns the relative difference between the target heading
  and the current heading. Positive direction is
  counter-clockwise, so a returned value of eg. 10 means that
  the robot should turn 10 degrees left to be on target.
  Similarly, a returned value of -20 would indicate that the
  robot would have to turn 20 degrees right to be on target.
*/

int getHeadingDiff(int targetHeading, int currentHeading) {
  int diff = (targetHeading - currentHeading + 360) % 360;
  if (diff >= 180) {
    return -360 + diff;
  }
  return diff;
}

int getCurrentHeading() {
  sensors_event_t mag_event;
  sensors_vec_t   orientation;
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Y, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    return (int) orientation.heading;
  }
}

byte getTurnSpeed(int headingDiff) {
  int turnSpeed = headingDiff;
  if (turnSpeed < 50) {
    turnSpeed = 50;
  }
  else if (turnSpeed > 200) {
    turnSpeed = 200;
  }
 return turnSpeed;
} 

byte getHeadingTolerance(int currentHeading, int targetHeading) {
  int deltaAngle;
  deltaAngle = getHeadingDiff(targetHeading, currentHeading);
  return 5 + abs(deltaAngle) / 10;
}

/*
  Returns degrees per second. 
*/

int getGyroOrientation() {
  sensors_event_t event; 
  gyro.getEvent(&event);
  
  return (int) (event.gyro.y * _RAD_TO_DEG);
}
