/*
 * MPL3115A2 Altitude/Pressure Sensor Library
 *
 * Copyright (C) 2015 Jonathan Martin <therouquinblanc@gmail.com>
 * Based on original work from:
 *   - A.Weiss, 7/17/2012
 *   - Nathan Seidle, 9/23/2013
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __MPL3115A2__
#define __MPL3115A2__

#include <Arduino.h>

class MPL3115A2
{
  public:
    MPL3115A2();

    //Sets the mode to Barometer
    void setModeBarometer();

    //Sets the mode to Altimeter
    void setModeAltimeter();

    //Enables the pressure and temp measurement event flags
    void enableEventFlags();

    //Returns the number of meters above sea level
    float readAltitude(bool oneShot);

    //Reads the current pressure in Pa
    //Unit must be set in barometric pressure mode
    float readPressure(bool oneShot, int altitude);

    float readTemp(bool oneShot);

    //Give me temperature in fahrenheit!
    float readTempF(bool oneShot);

    //Puts the sensor in standby mode
    //This is needed so that we can modify the major control registers
    void setModeStandby();

    //Puts the sensor in active mode
    //This is needed so that we can modify the major control registers
    void setModeActive();

    //Setup FIFO mode to one of three modes.
    void setFIFOMode(byte f_Mode);

    //Call with a rate from 0 to 7, ie 2^sampleRate samples.
    void setOversampleRate(byte sampleRate);

    // Set time between samples as 2^ST_Value seconds
    void setTimeStep(byte ST_Value);

    //Clears then sets the OST bit which causes the sensor to immediately take another reading
    void toggleOneShot(void);

    // Tell if MPL3115A2 is online on I2C bus
    bool online(void);

    // Reset MPL3115A2 module
    void reset(void);

    // Set MPL3115A2 to rise an interrupt when data is available
    void setDataOnInterrupt(void);

  private:
    byte IIC_Read(byte regAddr);
    void IIC_Write(byte regAddr, byte value);
};

#endif /* __MPL3115A2__ */
