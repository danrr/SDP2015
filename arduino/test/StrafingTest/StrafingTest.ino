#include "SDPArduino.h"
#include <Wire.h>
 byte msg;  // the command buffer

void setup(){
  SDPsetup();
  pinMode(8, OUTPUT);    // initialize pin 8 to control the radio
  digitalWrite(8, HIGH); // select the radio
  helloWorld();
}


void loop(){
  if (Serial.available()>=1){
    msg = (char)Serial.read();
    if (msg == '0'){
      motorAllStop();
      Serial.println("Motor off");
    }
    else if (msg == '1'){
      motorBreak(2);
      motorBreak(4);
      motorForward(5, 100);
      Serial.println("Strafe Left");
    }else if (msg == '2'){
      motorBreak(2);
      motorBreak(4);
      motorBackward(5, 100);
      Serial.println("Strafe Right");
    }
    else{
    }
  } 


}
