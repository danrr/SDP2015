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

/* Assign a unique ID to the sensors */

Adafruit_9DOF                  dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified  accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified    mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified        gyro = Adafruit_L3GD20_Unified(20);

/*
  Command class bytes. Set to gaming-related stuff
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

#define LEFT_DRIVE 2
#define RIGHT_DRIVE 4
#define BACK_DRIVE 3
#define LEFT_GRABBER 1
#define RIGHT_GRABBER 5


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
}

/*
  Main loop. Runs an endless loop of fetching commands from the serial
  comms, decoding them, and generally doing the stuff we're supposed to do.
*/

void loop() {
  fetchCommand();
  decodeCommand();
  respond();
}

/*
  Fetches a single command (command byte + data byte) from
  the serial stream if there is data available.
*/

void fetchCommand() {
  if (Serial.available() >= 2) {
    cmd = (char)Serial.read();
    data = (char)Serial.read();
    response = 0x00;
  }
}

void respond() {
  if (response) {
    Serial.write(response);
  }
  response = 0x00;
}

/*
  Move forward with the power specified. Currently, 0<=power<=100,
  but will probably make more sense to use the whole byte range
  0-255 in the future.
*/

void moveForward(byte power) {
  motorBackward(LEFT_DRIVE, power);
  motorForward(RIGHT_DRIVE, power);
}

/*
  Move backward with the power specified. Currently, 0<=power<=100,
  but will probably make more sense to use the whole byte range
  0-255 in the future.
*/

void moveBackward(byte power) {
  motorForward(LEFT_DRIVE, power); 
  motorBackward(RIGHT_DRIVE, power * 1);
}

void turnLeft(byte power) {
   motorForward(LEFT_DRIVE, power);
   motorForward(RIGHT_DRIVE, power);
 }
 
void turnRight(byte power) {
   motorBackward(LEFT_DRIVE, power);
   motorBackward(RIGHT_DRIVE, power);
 }

void closeGrabber() {
  cmd = 0x00;
  motorForward(LEFT_GRABBER, 100);
  motorBackward(RIGHT_GRABBER, 100);
  delay(500);
  motorAllStop();
  motorStop(LEFT_GRABBER);
  motorStop(RIGHT_GRABBER);  
}

void openGrabber() {
  cmd = 0x00;
  motorBackward(LEFT_GRABBER, 100);
  motorForward(RIGHT_GRABBER, 100);
  delay(500);
  motorAllStop();
  motorStop(LEFT_GRABBER);
  motorStop(RIGHT_GRABBER);  
}

/*
  Perform a kick. Takes a byte, 0<=power<=100, to determine the relative
  power of the kick.
  
  The function introduces delays into the loop, which obviosuly
  isn't good. We'll have to figure out a way for it to run without locking
  the rest of the Arduino system.
*/

void kick(byte power) {
  // reset the command byte so that we don't keep kicking.
  cmd = 0x00;
  
  // convert the power from a percentage to a ratio.
  float relativePower = (power/100.0); 
  
  //then perform the kick.
  kicker.write(90);
  delay(200);
  kicker.write(90 - (7 * relativePower));
  delay(500);
  kicker.write(90 + (30 * relativePower));
  delay(200);
  kicker.write(90 + (15 * relativePower));
  delay(200);
  kicker.write(90);
}

/*
  Decodes the current command byte, and executes the instruction given.
*/

void decodeCommand() {
   switch(cmd) {
     case _FORWARD:
       moveForward(data);
       break;
     case _BACKWARD:
       moveBackward(data);
       break;
     case _STOP:
       motorAllStop();
       break;
     case _KICK:
       kick(data);
       break; 
     case _HEARTBEAT:
       heartbeat(data);
       break; 
     case _GRABBER_OPEN:
       openGrabber();
       break;
     case _GRABBER_CLOSE:
       closeGrabber();
       break;
     case _TURN_LEFT:
       turnLeft(data);
       break;
     case _TURN_TO:
       turnToHeading(data * 2);
       break;
     case _TURN_RIGHT:
       turnRight(data);
       break;
     case _GET_HEADING:
       returnHeading();
       break;
     
   }
}

/*
  Sends a response to a received Hearbeat request by simply
  returning the request data. We could do something more sophisticated
  here, but for now it'll probably do.
*/

void heartbeat(byte _data) {
  cmd = 0x00;
  response = _data;
}

/* Initialises the IMU sensors. Taken (mostly) verbatim from the example program. */

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

void turnToHeading(short heading) {
  cmd = 0x00;
  short currentHeading;
  short diffHeading;
  do {
    currentHeading = getMagOrientation();
    diffHeading = currentHeading - heading;
    turnLeft((diffHeading / 2) + 20);
  } while (abs(diffHeading) > 5);
  motorAllStop();
}

void returnHeading() {
  cmd = 0x00;
  response = (byte) (getMagOrientation() / 2); 
}

short getMagOrientation() {
  sensors_event_t mag_event;
  sensors_vec_t   orientation;
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Y, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    return (short) orientation.heading;
  }
}

short getGyroOrientation() {
  sensors_event_t event; 
  gyro.getEvent(&event);
  
  return (short) event.gyro.z;
}
