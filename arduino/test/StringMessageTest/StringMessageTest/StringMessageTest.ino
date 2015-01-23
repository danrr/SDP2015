char inData[20];   // Allocate some space for the string
char inChar=-1;    // Where to store the character read
byte index = 0;    // Index into array; where to store the character

void setup() {                
  pinMode(8, OUTPUT);      // initialize pin 8 to control the radio
  digitalWrite(8, HIGH);   // select the radio
  Serial.begin(115200);    // start the serial port at 115200 baud (correct for XinoRF and RFu, if using XRF + Arduino you might need 9600)
}

// Accepts an arbitrary string and returns '0' if the param string matches the message, '1' otherwise
char comp(char* str) {
    while (Serial.available() > 0)
    {
        if(index < 19)
        {
            inChar = Serial.read();   // Read a character
            inData[index] = inChar;   // Store it
            index++;                  // Increment where to write next
            inData[index] = '\0';     // Null terminate the string
        }
    }

    if (strcmp(inData,str)  == 0) {
        for (int i=0;i<19;i++) {
            inData[i]=0;
        }
        index=0;
        return(0);
    }
    else {
        return(1);
    }
}

void loop()
{
    // Decide what to do when receiving specific messages
    if (Comp("FORWARD")==0) {
        Serial.write("Forward\n");
    }
    if (Comp("BACKWARD")==0) {
        Serial.write("Backward\n");
    }
}
