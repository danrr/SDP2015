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
int targetHeading;

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
#define _TURN_TO 'O'

#define _LEFT_DRIVE 2
#define _RIGHT_DRIVE 4
#define BACK_DRIVE 3
#define LEFT_GRABBER 1
#define RIGHT_GRABBER 5

#define _HEADING_TOLERANCE 1
#define _HEADING_CORRECTION_COEFFICIENT 0.5


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
  commands[1].functionPtr = &liftKicker;
  commands[2].functionPtr = &doKick;
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
  fetchCommand();
  if (cmd) {
    decodeCommand();
  }
  doStoredCommands();
  respond();
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
    response = 0x00;
  }
}

/*
  Decodes the current command byte, and executes the instruction given.
*/

void decodeCommand() {
  switch(cmd) {
    // movement commands
    case _FORWARD:
      commands[0].millis = millis();
      commands[0].functionPtr = &moveForward;
      commands[0].data = data;
      targetHeading = getCurrentHeading();
      break; 
    case _BACKWARD:
      commands[0].millis = millis();
      commands[0].functionPtr = &moveBackward;
      commands[0].data = data;
      targetHeading = getCurrentHeading();
      break; 
    case _STOP:
      commands[0].millis = millis();
      commands[0].functionPtr = &driveMotorStop;
      commands[0].data = data;
      break;

    // kick commands  
    case _KICK:
      current_millis = millis();
      commands[1].millis = current_millis;
      commands[2].millis = current_millis + 200;
      commands[3].millis = current_millis + 400;
      commands[1].data = commands[2].data = commands[3].data = data;
      break;

    // grabber commands
    case _GRABBER_OPEN:
      current_millis = millis();
      commands[4].millis = current_millis;
      commands[4].functionPtr = &startOpenGrabber;

      commands[5].millis = current_millis + 200;
      commands[5].functionPtr = &slowOpenGrabber;

      commands[6].millis = current_millis + 400;
      commands[6].functionPtr = &stopGrabber;

      commands[4].data = commands[5].data = commands[6].data = data;
      break;

    case _GRABBER_CLOSE:
      current_millis = millis();
      commands[4].millis = current_millis;
      commands[4].functionPtr = &startCloseGrabber;

      commands[5].millis = current_millis + 200;
      commands[5].functionPtr = &slowCloseGrabber;

      commands[6].millis = current_millis + 400;
      commands[6].functionPtr = &stopGrabber;

      commands[4].data = commands[5].data = commands[6].data = data;
      break;

     // case _TURN_LEFT:
     //   turnLeft(data);
     //   break;
     // case _TURN_TO:
     //   turnToHeading(data * 2);
     //   break;
     // case _TURN_RIGHT:
     //   turnRight(data);
     //   break;

    case _HEARTBEAT:
      heartbeat(data);
      break; 

    case _GET_HEADING:
      returnHeading();
      break;
     
   }
}

void doStoredCommands() {
  for (short i = 0; i < 7; i++) {
    if (commands[i].millis && commands[i].millis < millis()) {
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
  Serial.write(abs(headingDiff) / 2);
  if (headingDiff > (_HEADING_TOLERANCE)) {
    // turn left....
    Serial.write(0xFF);
    motorBackward(_LEFT_DRIVE, power - abs(headingDiff) * _HEADING_CORRECTION_COEFFICIENT);
    motorForward(_RIGHT_DRIVE, power);
  } else if (headingDiff < -(_HEADING_TOLERANCE)) {
    // turn right..
    Serial.write(0xFE);
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
  motorForward(_LEFT_DRIVE, power); 
  motorBackward(_RIGHT_DRIVE, power);
}

void driveMotorStop(byte data) {
  motorStop(_LEFT_DRIVE);
  motorStop(_RIGHT_DRIVE);
  voidCommand(0);
}

// void turnLeft(byte power) {
//    motorForward(_LEFT_DRIVE, power);
//    motorForward(_RIGHT_DRIVE, power);
//  }
 
// void turnRight(byte power) {
//    motorBackward(_LEFT_DRIVE, power);
//    motorBackward(_RIGHT_DRIVE, power);
//  }

//grabber commands
void startOpenGrabber(byte data) {
  motorForward(RIGHT_GRABBER, 100);
  motorBackward(LEFT_GRABBER, 100);
  voidCommand(4);
}

void slowOpenGrabber(byte data) {
  motorForward(RIGHT_GRABBER, 50);
  motorBackward(LEFT_GRABBER, 50);
  voidCommand(5);
}

void startCloseGrabber(byte data) {
  motorForward(LEFT_GRABBER, 100);
  motorBackward(RIGHT_GRABBER, 100);
  voidCommand(4);
}

void slowCloseGrabber(byte data) {
  motorForward(LEFT_GRABBER, 50);
  motorBackward(RIGHT_GRABBER, 50);
  voidCommand(5);
}

void stopGrabber(byte data) {
  motorStop(LEFT_GRABBER);
  motorStop(RIGHT_GRABBER);
  voidCommand(6);
}

//kicker commands
void liftKicker(byte power) {
  float relativePower = (power/100.0);
  kicker.write(90 - (7 * relativePower));
  voidCommand(1);
}

void doKick(byte power) {
  float relativePower = (power/100.0);   
  kicker.write(90 + (30 * relativePower));
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

/* Initialises the IMU sensors. Taken (mostly) verbatim from the example program. */



// void turnToHeading(short heading) {
//   cmd = 0x00;
//   short currentHeading;
//   short diffHeading;
//   do {
//     currentHeading = getMagOrientation();
//     diffHeading = currentHeading - heading;
//     turnLeft((diffHeading / 2) + 20);
//   } while (abs(diffHeading) > 5);
//   motorAllStop();
// }

void returnHeading() {
  response = (byte) (getCurrentHeading() / 2); 
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
  if (diff > 180) {
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

// short getGyroOrientation() {
//   sensors_event_t event; 
//   gyro.getEvent(&event);
  
//   return (short) event.gyro.z;
// }
