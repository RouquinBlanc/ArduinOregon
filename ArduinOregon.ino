/*
 MPL3115A2 Barometric Sensor Example
 By:  A.Weiss, 7/17/2012, 
      changes Nathan Seidle Sept 23rd, 2013,
      changes Jonathan Martin Aug 12th, 2015,
      
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 */
 
#include <avr/power.h>
#include <Wire.h>
#include <MPL3115A2.h>
#include <Oregon.h>
#include <Power.h>

/* DEBUG mode (enable for serial print) */
#include <Debug.h>

/* POWER REF: Adjust this value to your board's specific internal BG voltage */
#define REFERENCE_VOLTAGE 1077L
#define SLEEP_INTERRUPT 0
#define SLEEP_INTERRUPT_MODE RISING
Power power(REFERENCE_VOLTAGE); 

/* STATUS LED */
#define STATUS_PIN 13
#define STATUS_HIGH() digitalWrite(STATUS_PIN, HIGH)
#define STATUS_LOW() digitalWrite(STATUS_PIN, LOW)

/* MPL3115A2 config */
#define MPL_WAKE_PIN 2
#define MPL_TIME_STEP 6
#define MPL_SAMPLE_RATE 7
#define MPL_ALTITUDE 30 // Local altitude at home
MPL3115A2 mpl;

/* RF 433MHz config */
#define TRX_PIN  6
#define TRX_ID   0xAA
#define TRX_CHAN 2
#define VOLTAGE_THRESH 4.0
BTHR918N sender(TRX_PIN, TRX_ID, TRX_CHAN);

/* error occured */
bool error = 0;

void setup()
{
  DEBUG_begin(9600);  // start serial for output
  DEBUG_println("Booting MPL3115A2...");

  /* Some low power optimization */
  power_spi_disable(); // SPI not needed
  power_adc_disable(); // ADC not required
#if DEBUG == 0
  power_usart0_disable();
#endif
  
  Wire.begin();        // join i2c bus

  /* Reset MPL3115A2 */
  mpl.reset();
  delay(500);

  if(mpl.online())
    DEBUG_println("MPL3115A2 online!");
  else
  {
    DEBUG_println("No response - check connections");
  }

  // Configure the sensor
  mpl.setModeStandby();
  mpl.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  mpl.setOversampleRate(MPL_SAMPLE_RATE); // Set Oversample to the recommended 128
  mpl.setTimeStep(MPL_TIME_STEP); // 2 seconds
  mpl.enableEventFlags(); // Enable all three pressure and temp event flags
  mpl.setDataOnInterrupt();
  mpl.setModeActive();

  // Interrupt PIN setup
  pinMode(MPL_WAKE_PIN, INPUT);
  digitalWrite(MPL_WAKE_PIN, LOW);

  STATUS_LOW();
}

void loop()
{
  if (error) {
    // Blink the status LED
    STATUS_HIGH();
    delay(10);
    STATUS_LOW();
    delay(5000);

    // Do not execute the usual code
    return;
  }

  // SLEEP UNTIL DATA READY
  power.deepSleepUntilInterrupt(SLEEP_INTERRUPT, SLEEP_INTERRUPT_MODE);

  /* Activity */

  // Retrieve pressure
  float pressure = mpl.readPressure(0, MPL_ALTITUDE);
  DEBUG_print("Pressure(Pa):");
  DEBUG_println(pressure);

  // Retrieve temperature
  float temperature = mpl.readTemp(0);
  DEBUG_print("Temp(c):");
  DEBUG_println(temperature);

  // Get Voltage:
  float voltage = power.getVoltage();
  DEBUG_print("Voltage(v):");
  DEBUG_println(voltage);

  // Emit temperature as an Oregon sensor
  STATUS_HIGH();
  sender.send(temperature, 0, pressure, voltage > VOLTAGE_THRESH);
  STATUS_LOW();
  
  DEBUG_println("-----");
} /* loop */

