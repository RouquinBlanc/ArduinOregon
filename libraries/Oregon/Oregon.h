/*
 * Oregon Scientific sensor emulation
 *
 * Copyright (C) 2013 olivier.lebrun@gmail.com
 * Modified by Jonathan Martin <therouquinblanc@gmail.com>, August 2015
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

#ifndef __OREGON_H__
#define __OREGON_H__

#include <Arduino.h>

class Oregon
{
  public:
    Oregon(int dataPin);

  protected:
    int _dataPin;

    inline void setId(byte id);
    inline void setChannel(byte channel);
    inline void setType(byte b0, byte b1);

    inline void SEND_HIGH();
    inline void SEND_LOW();
    inline void sendZero(void);
    inline void sendOne(void);
    inline void sendQuarterMSB(const byte data);
    inline void sendQuarterLSB(const byte data);

    inline void sendData(byte *data, byte size);
    inline void sendPreamble(void);
    inline void sendSync(void);

    void sendOregon(byte *data, byte size);

    void setBatteryLevel(byte *data, bool correct);
    void setTemperature(byte *data, float temp);
    void setHumidity(byte* data, byte hum);
    void setPressure(byte *data, float pres);

    int Sum(byte count, const byte* data);

    void _send(byte *data, int size);

    virtual byte *getData(void) = 0;
    virtual void sendPostamble(void) = 0;
    virtual void calculateAndSetChecksum(byte* data) = 0;
};

class THN132N: public Oregon
{
  public:
    THN132N(int dataPin, byte id, byte channel);

    void send(float temperature, bool battery);

  protected:
    byte *getData(void);
    void sendPostamble(void);
    void calculateAndSetChecksum(byte* data);

  private:
    byte messageBuffer[8];
};

class THGR2228N: public Oregon
{
  public:
    THGR2228N(int dataPin, byte id, byte channel);

    void send(float temperature, float humidity, bool battery);

  protected:
    byte *getData(void);
    void sendPostamble(void);
    void calculateAndSetChecksum(byte* data);

  private:
    byte messageBuffer[9];
};

class BTHR918N: public Oregon
{
  public:
    BTHR918N(int dataPin, byte id, byte channel);

    void send(float temperature, float humidity, float pressure, bool battery);

  protected:
    byte *getData(void);
    void sendPostamble(void);
    void calculateAndSetChecksum(byte* data);

  private:
    byte messageBuffer[11];

};

#endif /* __OREGON_H__ */
