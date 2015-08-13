#ifndef __POWER_H__
#define __POWER_H__

#include <Arduino.h>

class Power
{
    public:
        Power(long ref);

        float getVoltage(void);

        void deepSleep(void);

        void deepSleepUntil(int millis);

    private:
        long _ref;
};

#endif /* __POWER_H__ */