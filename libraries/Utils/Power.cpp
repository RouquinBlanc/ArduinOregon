

#include <Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "Power.h"
#include <Debug.h>

Power::Power(long ref)
{
    _ref = ref;
}

float Power::getVoltage()
{
  power_adc_enable(); // ADC required

  // REFS0 : Selects AVcc external reference
  // MUX3 MUX2 MUX1 : Selects 1.1V (VBG)
  ADMUX = bit (REFS0) | bit (MUX3) | bit (MUX2) | bit (MUX1);
  ADCSRA |= bit( ADSC );  // start conversion

  while (ADCSRA & bit (ADSC)) { }  // wait for conversion to complete
  float results = (((_ref * 1024) / ADC) + 5) / 1000.0;

  power_adc_disable(); // ADC not required

  return results;

}

/* ISR called on INTERRUPT */
void wakeUpNow(void) { /* nothing to do, just wake up */ }

void Power::deepSleep()
{
  DEBUG_println("Going to sleep...");
  DEBUG_delay(100);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  attachInterrupt(0, wakeUpNow, RISING);

  // Turn off brown-out
  noInterrupts ();           // timed sequence follows
  sleep_enable();

  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS);
  interrupts ();             // guarantees next instruction executed
  sleep_cpu ();              // sleep within 3 clock cycles of above

  // SLEEPING

  sleep_disable();
  detachInterrupt(0);
  DEBUG_delay(100);
}
