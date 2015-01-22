#include "SDPArduino.h"
#include <Wire.h>
int i = 0;
 byte msg;  // the command buffer

void setup(){
  SDPsetup();
  pinMode(8, OUTPUT);    // initialize pin 8 to control the radio
  digitalWrite(8, HIGH); // select the radio
  helloWorld();
  

}

void loop(){
  if (Serial.available()>=0){
    msg = (char)Serial.read();
    if (msg == '1'){
      digitalWrite(13, LOW);
      motorForward(1, 100);
      motorForward(2, 100);
      Serial.print("Doge LED OFF");
      delay(2500);

    }
    else if (msg == '1'){
      digitalWrite(13, HIGH);
      motorStop(1);
      motorStop(2);
      Serial.print("Doge LED ON");
    }else if (msg == '2'){
      digitalWrite(13, HIGH);
      motorBackward(1, 50);
      motorBackward(2, 50);
      Serial.print("Doge LED ON");
    }
    else{
      Serial.print(msg);
    }
  } 


}
