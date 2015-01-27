/*
  Group 14 Arduino Code
*/

#include "SDPArduino.h"
#include <Wire.h>
#include <Servo.h>

byte cmd, data;
Servo kicker;

/*
  Command class bytes. Set to the dead-simple A, B, C, D, E
  now for debugging purposes, but will change to something
  more professional-looking in the future.
*/

#define _FORWARD 'A'
#define _BACKWARD 'B' 
#define _STOP 'C'
#define _KICK 'D'
#define _HEARTBEAT 'E'

/*
  Setup function. Performs the standard SDPsetup routine supplied by
  the course staff. Also sets up the kicker servo.
*/

void setup() {
  SDPsetup();
  // attach the kicker servo to pin 3, with 600us to 2400us pulses.
  kicker.attach(3, 600, 2400); 
}

/*
  Main loop. Runs an endless loop of fetching commands from the serial
  comms, decoding them, and generally doing the stuff we're supposed to do.
*/

void loop() {
  fetchCommand();
  decodeCommand();
}

/*
  Fetches a single command (command byte + data byte) from
  the serial stream if there is data available.
*/

void fetchCommand() {
  if (Serial.available() >= 2) {
    cmd = (char)Serial.read();
    data = (char)Serial.read();
  }
}

/*
  Move forward with the power specified. Currently, 0<=power<=100,
  but will probably make more sense to use the whole byte range
  0-255 in the future.
*/

void moveForward(byte power) {
  motorForward(2, power * 0.7); // 70% scaling on the right motor.
  motorBackward(4, power);
}

/*
  Move backward with the power specified. Currently, 0<=power<=100,
  but will probably make more sense to use the whole byte range
  0-255 in the future.
*/

void moveBackward(byte power) {
  motorForward(4, power); 
  motorBackward(2, power * 1); // 70% scaling on the right motor. 
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
   } 
}

/*
  Sends a response to a received Hearbeat request by simply
  returning the request data. We could do something more sophisticated
  here, but for now it'll probably do.
*/

void heartbeat(byte response) {
  cmd = '\0';
  Serial.write(response);
}
