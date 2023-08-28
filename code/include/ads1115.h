/*
ADS1115 glue code for ACS758 current sensor and ACPL-C870 isolated voltage measurement IC

(c)2023 Patrick Prasse

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your
  contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures
  that legally restrict others from doing anything the license permits.
*/

#ifndef __ADS1115_H__
#define __ADS1115_H__

#include <Arduino.h>

#define VOLTAGE_INTERVAL_MS 1000

const uint8_t ADS1115address = B01001010;

bool set_mA_per_mV(uint32_t);
uint32_t ads1115_setup();
void ads1115_isr();

void ads1115_loop();

double BusVoltage();
double Current();
double Power();

double AttinyTemperature();

uint16_t bitFlags();
void setBitFlags(uint16_t);

bool haveAlert();
bool relayOn();


#endif