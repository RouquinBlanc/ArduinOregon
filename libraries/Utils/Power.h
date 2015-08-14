#ifndef __POWER_H__
#define __POWER_H__

#include <Arduino.h>

class Power
{
    public:
        /**
         * Initialise power module.
         * @param ref   Reference voltage of the board.
         */
        Power(long ref);

        /**
         * Get board current Voltage.
         * @return voltage in volts.
         */
        float getVoltage(void);

        /**
         * Power Down Sleep mode until an interrupt arrives.
         * @param errupt    Interrupt number (0 or 1 on Arduino Uno/Nano)
         * @param mode      Interrupt mode (ex: RISING)
         */
        void deepSleepUntilInterrupt(int errupt, int mode);

    private:
        long _ref;
};

#endif /* __POWER_H__ */