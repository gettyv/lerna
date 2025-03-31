#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>


class Encoder {
    private:
        const uint8_t pinA;
        const uint8_t pinB;
        const uint16_t pulsesPerRev;
        const unsigned long velocity_update_period_us;
        
        volatile long position = 0;
        volatile bool lastStateA = false;
        volatile float velocity = 0.0;
        volatile long lastPosition = 0;
        
        bool useTimerThree = false;  // Flag to select Timer3 instead of Timer1
        
        static void isrWrapper();
        static void timer1IsrWrapper();
        static void timer3IsrWrapper();
        
        void handleInterrupt();
        void updateVelocity();
    
    public:
        Encoder(uint8_t encoderPinA, uint8_t encoderPinB, uint16_t ppr, unsigned long velocity_update_period_us, bool useTimer3 = false);
        
        void begin();
        
        float getVelocity();
        
        long getPosition();
        
        static Encoder* instance;
};
#endif // ENCODER_H