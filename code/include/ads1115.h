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

struct bitflags_struct
{
    uint16_t factory_reset_bit : 1;
    uint16_t existing_relay_state : 1;  // not settable
// RELAY-BITS
    uint16_t relay_trigger_pol : 1;
    uint16_t relay_trigger_busul : 1;
    uint16_t relay_trigger_busol : 1;
    uint16_t relay_trigger_ucurr : 1;
    uint16_t relay_trigger_ocurr : 1;
    uint16_t relay_trigger_tmpol : 1;
// /RELAY-BITS


    uint16_t adc_range : 1;
    uint16_t temp_compensation_enabled : 1;
// ALERT-BITS
    uint16_t pol : 1;
    uint16_t busul : 1;
    uint16_t busol : 1;
    uint16_t ucurr : 1;
    uint16_t ocurr : 1;
    uint16_t tmpol : 1;
// /ALERT-BITS
} __attribute__((packed));

typedef union {
  struct bitflags_struct bits;
  uint16_t uint16;
} bitflags;


#define SETTABLE_BITFLAGS_MASK (uint16_t)0xfffd
#define RELAY_BITS (uint16_t)0xFC
#define ALERT_BITS (uint16_t)0xFC00

#define VOLTAGE_INTERVAL_MS 1000

const uint8_t ADS1115address = B01001010;

uint32_t ads1115_setup();
bool ads1115_setupContinuousAmpereSampling();
bool ads1115_setupVoltageSampling();
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