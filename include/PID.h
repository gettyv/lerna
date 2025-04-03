#ifndef PID_H
#define PID_H

class PID {
    public:
        PID(float Kp, float Ki, float Kd);

        float step(float reading);

        void setRef(float ref);

    private:
        float accumulator;
        float error;
        float lastError;
        float lastUpdateTime = -1;
        float ref;

        float Kp;
        float Ki;
        float Kd;

};




#endif