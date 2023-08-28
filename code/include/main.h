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


#define SETTABLE_BITFLAGS_MASK (uint16_t)0xfd    // factory_reset_bit + RELAY-BITS
#define RELAY_BITS (uint16_t)0xFC
#define ALERT_BITS (uint16_t)0xFC00


void RedLED(bool value);
void GreenLED(bool value);


#define GREENLED_PIN_BITMAP PIN7_bm
#define REDLED_PIN_BITMAP PIN6_bm
#define RELAY_PIN_BITMAP PIN5_bm

// Sequences used to indicate error on RED led
// 7 flashes (0xAA=4 fast flashes)
const static uint32_t err_ADS1115_testConnection = 0xAAA8000F;


// This structure is held in EEPROM, it has the same register/values
// as the INA228 chip and is used to set the INA228 chip to the correct parameters on power up

// On initial power up (or EEPROM clear) these parameters are read from the INA228 chip
// to provide defaults.  Some values are overridden in code (like ADC_CONFIG and CONFIG)
// to configure to our prescribed needs.
struct eeprom_regs
{
  uint16_t git_version_b1;
  bitflags bitflags;
  /* @deprecated */
  int16_t R_SHUNT_CAL;    // Shunt Calibration
  /* @deprecated */
  uint16_t R_SHUNT_TEMPCO; // Shunt Temperature Coefficient
//  uint16_t R_DIAG_ALRT;
  double bus_overcurrent;
  double bus_undercurrent;
  double bus_overvoltage;
  double bus_undervoltage;
  double temp_limit;
  double power_limit;

  uint16_t shunt_max_current;
  uint16_t shunt_millivolt;

  // Resistance of SHUNT in OHMS
  // With ACS758 this is just a fake. 
  // ACS758 hall sensor produces 2.5V at +200A, so 2.5V / 200A
  double RSHUNT;

  uint16_t batterycapacity_amphour;
  double fully_charged_voltage;
  double tail_current_amps;
  double charge_efficiency_factor;

  uint16_t git_version_b2;
};


typedef union
{
  double dblvalue;
  uint16_t word[2];
} DoubleUnionType;
