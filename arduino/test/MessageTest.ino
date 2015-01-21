/*
  BlinkwithRadio
  Turns on an LED on for one second, then off for one second, repeatedly.
  Turns on the SRF radio and reports LED status.
  This example code is based on Blink, which is in the public domain.
 */
 
 byte msg;  // the command buffer
 
void setup() {                
  pinMode(13, OUTPUT);   // initialize pin 13 as digital output (LED)
  pinMode(8, OUTPUT);    // initialize pin 8 to control the radio
  digitalWrite(8, HIGH); // select the radio
  Serial.begin(115200);    // start the serial port at 115200 baud (correct for XinoRF and RFu, if using XRF + Arduino you might need 9600)
}
void loop() {

  if (Serial.available()>=1){
    msg = (char)Serial.read();
    if (msg == '0'){
      digitalWrite(13, LOW);
      Serial.print("Doge LED OFF");
    }
    else if (msg == '1'){
      digitalWrite(13, HIGH);
      Serial.print("Doge LED ON");
    }else{
      Serial.print(msg);
    }
  } 
}


