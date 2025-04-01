#ifndef PID_H
#define PID_H

class PID {
    public:
        PID(float Kp, float Ki, float Kd);

        float step(float reading);

        float setRef(float ref);

    private:
        float accumulator;
        float error;
        float lastUpdateTime;
};




#endif