#include <Arduino.h>


enum INA_REGISTER : uint8_t
{
  CONFIG = 0,
  ADC_CONFIG = 1,
  SHUNT_CAL = 2,    //Shunt Calibration
  SHUNT_TEMPCO = 3, //Shunt Temperature Coefficient
  VSHUNT = 4,       //Shunt Voltage Measurement 24bit
  VBUS = 5,         //Bus Voltage Measurement 24bit
  DIETEMP = 6,
  CURRENT = 7,   //Current Result 24bit
  POWER = 8,     //Power Result 24bit
  ENERGY = 9,    //Energy Result 40bit
  CHARGE = 0x0A, //Charge Result 40bit
  DIAG_ALRT = 0x0b,
  SOVL = 0x0c, //Shunt Overvoltage Threshold
  SUVL = 0x0d, //Shunt Undervoltage Threshold
  BOVL = 0x0e, //Bus Overvoltage Threshold
  BUVL = 0x0f, //Bus Undervoltage Threshold
  TEMP_LIMIT = 0x10,
  PWR_LIMIT = 0x11,
  MANUFACTURER_ID = 0xFE,
  DIE_ID = 0xFF
};

enum DIAG_ALRT_FIELD : uint16_t
{
  ALATCH = 15,
  CNVR = 14,
  SLOWALERT = 13,
  APOL = 12,
  ENERGYOF = 11,
  CHARGEOF = 10,
  MATHOF = 9,
  RESERVED = 8,
  TMPOL = 7,
  SHNTOL = 6,
  SHNTUL = 5,
  BUSOL = 4,
  BUSUL = 3,
  POL = 2,
  CNVRF = 1,
  MEMSTAT = 0
};


const uint16_t ALL_ALERT_BITS = (bit(DIAG_ALRT_FIELD::TMPOL) |
                                 bit(DIAG_ALRT_FIELD::SHNTOL) |
                                 bit(DIAG_ALRT_FIELD::SHNTUL) |
                                 bit(DIAG_ALRT_FIELD::BUSOL) |
                                 bit(DIAG_ALRT_FIELD::BUSUL) |
                                 bit(DIAG_ALRT_FIELD::POL));

void RedLED(bool value);
void GreenLED(bool value);


#define GREENLED_PIN_BITMAP PIN7_bm
#define REDLED_PIN_BITMAP PIN6_bm
#define RELAY_PIN_BITMAP PIN5_bm

// Sequences used to indicate error on RED led
// Single flash
const static uint32_t err_INA228Missing = 0xF000000F;
// Two flashes
const static uint32_t err_InitialConfigure = 0xF0F0000F;
// 3 flashes
const static uint32_t err_WrongChip = 0xF0F0F00F;
// 4 flashes (0xCC=2 flashes)
const static uint32_t err_WriteConfig = 0xCCCC000F;
// 5 flashes (0xC0=1 short flash)
const static uint32_t err_WriteADCConfig = 0xCCCCC00F;
// 6 flashes
const static uint32_t err_WriteRegisters = 0xCCCCCC0F;
// 7 flashes (0xAA=4 fast flashes)
const static uint32_t err_INA228Reset = 0xAAA8000F;
// 8 flashes  10101010101010100000000000001111
const static uint32_t err_CheckSumErr = 0xAAAA000F;
// 9 flashes 00101010101010101010000000001111
const static uint32_t err_ResetChargeEnergyRegisters = 0x2AAAA00F;




// This structure is held in EEPROM, it has the same register/values
// as the INA228 chip and is used to set the INA228 chip to the correct parameters on power up

// On initial power up (or EEPROM clear) these parameters are read from the INA228 chip
// to provide defaults.  Some values are overridden in code (like ADC_CONFIG and CONFIG)
// to configure to our prescribed needs.
struct eeprom_regs
{
  uint16_t bitflags;
  /* @deprecated */
  uint16_t R_SHUNT_CAL;    // Shunt Calibration
  /* @deprecated */
  uint16_t R_SHUNT_TEMPCO; // Shunt Temperature Coefficient
//  uint16_t R_DIAG_ALRT;
  double bus_overcurrent;
  double bus_undercurrent;
  double bus_overvoltage;
  double bus_undervoltage;
  double temp_limit;
  double power_limit;

  // Holds what alert events trigger the relay to turn on/high
  // uses the same values/mapping as enum DIAG_ALRT_FIELD
  uint16_t relay_trigger_bitmap;

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
};
