#include <Arduino.h>

#include "Oregon.h"
#include <Debug.h>
 
const unsigned long TIME = 512;
const unsigned long TWOTIME = TIME*2;

/*********************************************
 * Oregon Main class - Generic functionality *
 *********************************************/

Oregon::Oregon(int pin)
{
  _dataPin = pin;
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, LOW);
}

void Oregon::setId(byte id)
{
  byte *data = getData();
  data[3] = id;
}

void Oregon::setChannel(byte channel)
{
  if (channel > 3) channel = 3;
  channel <<= 4;
  byte *data = getData();
  data[2] = channel;
}

void Oregon::setType(byte b0, byte b1)
{
  byte *data = getData();
  data[0] = b0;
  data[1] = b1;
}

inline void Oregon::SEND_HIGH() { digitalWrite(_dataPin, HIGH); }
inline void Oregon::SEND_LOW() { digitalWrite(_dataPin, LOW); }

/**
 * \brief    Send logical "0" over RF
 * \details  azero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first
 */
inline void Oregon::sendZero(void)
{
  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
}

/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first
 */
inline void Oregon::sendOne(void)
{
   SEND_LOW();
   delayMicroseconds(TIME);
   SEND_HIGH();
   delayMicroseconds(TWOTIME);
   SEND_LOW();
   delayMicroseconds(TIME);
}

/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/

/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void Oregon::sendQuarterMSB(const byte data)
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}

/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void Oregon::sendQuarterLSB(const byte data)
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}

/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
void Oregon::sendData(byte *data, byte size)
{
  for(byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}

/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
void Oregon::sendOregon(byte *data, byte size)
{
    sendPreamble();
    //sendSync();
    sendData(data, size);
    sendPostamble();
}

/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void Oregon::sendPreamble(void)
{
  byte PREAMBLE[]={0xFF,0xFF};
  sendData(PREAMBLE, 2);
}

/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void Oregon::sendSync(void)
{
  sendQuarterLSB(0xA);
}

/**
 * \brief    Set the sensor battery level
 * \param    data       Oregon message
 * \param    level      Battery level (0 = low, 1 = high)
 */
void Oregon::setBatteryLevel(byte *data, bool ok)
{
  if(!ok) data[4] |= 0x0C;
  else data[4] &= ~0x0C;
}

/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
void Oregon::setTemperature(byte *data, float temp)
{
  // Set temperature sign
  if(temp < 0)
  {
    data[6] = 0x08;
    temp *= -1;
  }
  else
  {
    data[6] = 0x00;
  }

  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt/10 - (float)td) * 10);

  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);

  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;

  // Set temperature float part
  data[4] |= (tempFloat << 4);
}

/**
 * \brief    Set the sensor humidity
 * \param    data       Oregon message
 * \param    hum        the humidity
 */
void Oregon::setHumidity(byte* data, byte hum)
{
    data[7] = (hum/10);
    data[6] |= (hum - data[7]*10) << 4;
}

/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
void Oregon::setPressure(byte *data, float pres)
{
  if ((pres > 850) && (pres < 1100)) {
    data[8] = (int)round(pres) - 856;
    data[9] = 0xC0;
  }
}

/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
int Oregon::Sum(byte count, const byte* data)
{
  int s = 0;

  for(byte i = 0; i<count;i++)
  {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }

  if(int(count) != count)
    s += (data[count]&0xF0) >> 4;

  return s;
}

void Oregon::_send(byte *data, int size)
{
  // Calculate the checksum
  calculateAndSetChecksum(data);

#if DEBUG
  DEBUG_print("TX: ");
  for (byte i = 0; i < size; ++i)   {
    DEBUG_print(data[i] >> 4, HEX);
    DEBUG_print(data[i] & 0x0F, HEX);
  }
  DEBUG_println();
#endif

  // Send the Message over RF
  sendOregon(data, size);

  // Send a "pause"
  SEND_LOW();
  delayMicroseconds(TWOTIME*8);

  // Send a copie of the first message (v2.1 protocol)
  sendOregon(data, size);

  SEND_LOW();
}

/**********************************************
 * THN132N class - Temperature only    sensor *
 **********************************************/

THN132N::THN132N(int pin, byte id, byte channel)
: Oregon(pin)
{
  setId(id);
  setChannel(channel);
  setType(0xEA, 0x4C);
}

byte *THN132N::getData()
{
  return messageBuffer;
}

void THN132N::send(float temperature, bool battery)
{
  // Setup message
  setTemperature (messageBuffer, temperature);
  setBatteryLevel(messageBuffer, battery);

  _send(messageBuffer, sizeof(messageBuffer));
}

inline void THN132N::sendPostamble(void)
{
  sendQuarterLSB(0x00);
}

void THN132N::calculateAndSetChecksum(byte* data)
{
    int s = ((Sum(6, data) + (data[6]&0xF) - 0xa) & 0xff);

    data[6] |=  (s&0x0F) << 4;     data[7] =  (s&0xF0) >> 4;
}



/*************************************************
 * THGR2228N class - Temperature/Humidity sensor *
 *************************************************/

THGR2228N::THGR2228N(int pin, byte id, byte channel)
: Oregon(pin)
{
  setId(id);
  setChannel(channel);
  setType(0x1A, 0x2D);
}

byte *THGR2228N::getData()
{
  return messageBuffer;
}

void THGR2228N::send(float temperature, float humidity, bool battery)
{
  // Setup message
  setTemperature (messageBuffer, temperature);
  setHumidity    (messageBuffer, humidity);
  setBatteryLevel(messageBuffer, battery);

  _send(messageBuffer, sizeof(messageBuffer));
}

void THGR2228N::sendPostamble(void)
{
  byte POSTAMBLE[]={0x00};
  sendData(POSTAMBLE, 1);
}

void THGR2228N::calculateAndSetChecksum(byte* data)
{
    data[8] = ((Sum(8, data) - 0xa) & 0xFF);
}



/**********************************************
 * BTHR918N class - Temp/Humidity/Baro sensor *
 **********************************************/

BTHR918N::BTHR918N(int pin, byte id, byte channel)
: Oregon(pin)
{
  setId(id);
  setChannel(channel);
  setType(0x5A, 0x6D);
}

byte *BTHR918N::getData()
{
  return messageBuffer;
}

void BTHR918N::send(float temperature, float humidity, float pressure, bool battery)
{
  // Setup message
  setTemperature (messageBuffer, temperature);
  setHumidity    (messageBuffer, humidity);
  setPressure    (messageBuffer, pressure);
  setBatteryLevel(messageBuffer, battery);

  _send(messageBuffer, sizeof(messageBuffer));
}

void BTHR918N::sendPostamble(void)
{
  byte POSTAMBLE[]={0x00};
  sendData(POSTAMBLE, 1);
}

void BTHR918N::calculateAndSetChecksum(byte* data)
{
    data[10] = ((Sum(10, data) - 0xa) & 0xFF);
}
