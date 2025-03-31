#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

/*
ENCODER CLASS

Notably, the encoder class will have to be split up. This is due to my
inexperience with class instances and that whole can of warms. The state
of both encoders will be global variables that will be changed with the
help of this class.

Most method calls will require passing in the current count,
and will purely handle so


*/
class Encoder {

    public:
        Encoder();
        
        void begin();

        int encoderUpdateInterrupt(bool lastStateA, 
            bool currentStateA, bool currentStateB);

        void updateVelocity(long currentPosition,
            long lastPosition, uint32_t currentUpdateTime, uint32_t lastUpdateTime);
        
        float getVelocity();

    private:
    // Velocity will be handled by the object as we can just keep passing
    // the positions into it
        volatile float velocity = 0.0;
};
#endif // ENCODER_H