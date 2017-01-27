
#include <wiringPi.h>

class ShiftReg {
    public:
        ShiftReg(int clockPin, int datapin, int latchPin);
        ~ShiftReg();
        void sendData(unsigned char dataOut);

    private:
        int myDataPin;
        int myClockPin;
        int myLatchPin;
}
