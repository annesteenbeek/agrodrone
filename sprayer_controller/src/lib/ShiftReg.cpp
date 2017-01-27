
ShiftReg::ShiftReg(int clockPin, int dataPin, int latchPin) {
    myClockPin = clockPin;
    myDataPin = dataPin;
    myLatchPin = latchPin;

    wiringPiSetup();
   
    //TODO catch and report any errors occuring when setting pins
    pinMode(myClockPin, OUTPUT);
    pinMode(myDataPin, OUTPUT);
    pinMode(myLatchPin, OUTPUT);
}

void ShiftReg::sendData(unsigned char dataOut) {
    // set latch to low to allow input
    digitalWrite(myLatchPin, 0);
    int pinState;

    //clear everything out just in case to
    //prepare shift register for bit shifting
    digitalWrite(myDataPin, 0);
    digitalWrite(myClockPin, 0);

    //NOTICE THAT WE ARE COUNTING DOWN in our for loop
    //This means that %00000001 or "1" will go through such
    //that it will be pin Q0 that lights. 
    for (int i=7; i>=0; i--)  {
        digitalWrite(myClockPin, 0);

        //if the value passed to myDataOut and a bitmask result 
        // true then... so if we are at i=6 and our value is
        // %11010100 it would the code compares it to %01000000 
        // and proceeds to set pinState to 1.
        if ( dataOut & (1<<i) ) {
        pinState= 1;
        } else {  
        pinState= 0;
        }

        //Sets the pin to HIGH or LOW depending on pinState
        /* digitalWrite(myDataPin, pinState); */
        digitalWrite(myDataPin, pinState);
        //register shifts bits on upstroke of clock pin  
        digitalWrite(myClockPin, 1);
        //zero the data pin after shift to prevent bleed through
        digitalWrite(myDataPin, 0);
    }
    //stop shifting
    digitalWrite(myClockPin, 0);
    digitalWrite(myLatchPin, 1);
}

ShiftReg::~ShiftReg(){
    sendToShift(0); // set all pins low, should be hard reset
}
