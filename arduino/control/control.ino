/*
  Group 14 Arduino Code
*/

#include "SDPArduino.h"
#include <Wire.h>
#include <Servo.h>

byte cmd, data;
Servo kicker;

/*
  Command class bytes. Set to the dead-simple A, B, C, D
  now for debugging purposes, but will change to something
  more professional-looking in the future.
*/

#define _FORWARD 'A'
#define _BACKWARD 'B' 
#define _STOP 'C'
#define _KICK 'D' 

/*
  Setup function. Performs the standard SDPsetup routine supplied by
  the course staff. Also sets up the kicker servo.
*/

void setup() {
  SDPsetup();
  helloWorld();
  
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
  Fetches a single command from the serial comms (in the shape of a byte)
  and resets the current command byte if no new orders have been issued.
  
  We'll have to change this slightly to enable variable speed etc.
*/

void fetchCommand() {
  cmd = '\0';
  if (Serial.available() >= 1) {
    cmd = (char)Serial.read();
  }
}

/*
  Move forward with the power specified. Currently, 0<=power<=100,
  but will probably make more sense to use the whole byte range
  0-255 in the future.
  
  The function doesn't use the power value supplied to it just yet,
  because having to specify the power would complicate comms debugging.
*/

void moveForward(byte power) {
  motorForward(2, 100);
  motorBackward(4, 100);
}

/*
  Move backward with the power specified. Currently, 0<=power<=100,
  but will probably make more sense to use the whole byte range
  0-255 in the future.
  
  The function doesn't use the power value supplied to it just yet,
  because having to specify the power would complicate comms debugging.
*/

void moveBackward(byte power) {
  motorForward(4, 100);
  motorBackward(2, 100);
}

/*
  Perform a kick. Takes no arguments (but maybe should in the future,
  to enable better control of where the ball ends up going?)
  
  The function also introduces delays into the loop, which obviosuly
  isn't good. We'll have to figure out a way for it to run without locking
  the rest of the Arduino system.
*/

void kick() {
  kicker.write(90);
  delay(200);
  kicker.write(83);
  delay(500);
  kicker.write(120);
  delay(200);
  kicker.write(105);
  delay(200);
  kicker.write(90);
}

/*
  Decodes the current command byte, and executes the instruction given.
*/

void decodeCommand() {
   switch(cmd) {
     case _FORWARD:
       moveForward(100);
       break;
     case _BACKWARD:
       moveBackward(100);
       break;
     case _STOP:
       motorAllStop();
       break;
     case _KICK:
       kick();
       break; 
   } 
}
