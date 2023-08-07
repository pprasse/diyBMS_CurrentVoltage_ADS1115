/*
 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

DIYBMS V4.0
ISOLATED CURRENT & VOLTAGE MONITORING SYSTEM BASEDO ON ADS1115 / ACS758 / ACPL-C870

(c)2023 Patrick Prasse

based on https://github.com/stuartpittaway/diyBMS-CurrentShunt
(c)2021 Stuart Pittaway

COMPILE THIS CODE USING PLATFORM.IO

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

** COMMERCIAL USE AND RESALE PROHIBITED **

*/

// ATTINY1614 (tinyAVR® 1-series of microcontrollers)
// http://ww1.microchip.com/downloads/en/DeviceDoc/ATtiny1614-data-sheet-40001995A.pdf

// MODBUS Protocol
// https://www.ni.com/en-gb/innovations/white-papers/14/the-modbus-protocol-in-depth.html

#if !defined(MODBUSDEFAULTBAUDRATE)
#error MODBUSDEFAULTBAUDRATE must be defined
#endif
#if !defined(MODBUSBASEADDRESS)
#error MODBUSBASEADDRESS must be defined
#endif
#if !defined(MODBUSSERIALCONFIG)
#error MODBUSSERIALCONFIG must be defined
#endif

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <EEPROM.h>

#include <FastCRC.h>
#include "debug.h"
#include "ads1115.h"


FastCRC16 CRC16;

#include "main.h"
#include "settings.h"

#include "EmbeddedFiles_Defines.h"

#include "SimpleModbusSlave.h"

typedef union
{
  double dblvalue;
  uint16_t word[2];
} DoubleUnionType;

const uint16_t loop_delay_ms = 2000;

uint32_t milliamphour_out_lifetime = 0;
uint32_t milliamphour_in_lifetime = 0;

uint32_t daily_milliamphour_out = 0;
uint32_t daily_milliamphour_in = 0;

uint32_t milliamphour_out = 0;
uint32_t milliamphour_in = 0;

uint32_t milliamphour_out_offset = 0;
uint32_t milliamphour_in_offset = 0;

uint16_t ModBusBaudRate = MODBUSDEFAULTBAUDRATE;

uint8_t ModbusSlaveAddress = MODBUSBASEADDRESS;

volatile bool config_dirty = false;

uint8_t max_soc_reset_counter = 0;
uint8_t soc_reset_counter = 0;
int32_t last_charge_coulombs = 0;

unsigned long timer = 0;

eeprom_regs registers;

volatile bool wdt_triggered = false;
volatile uint16_t wdt_triggered_count;

uint16_t CalculateSOC()
{
  double milliamphour_in_scaled = ((double)milliamphour_in / 100.0) * registers.charge_efficiency_factor;
  double milliamphour_batterycapacity = 1000.0 * (uint32_t)registers.batterycapacity_amphour;
  double difference = milliamphour_in_scaled - milliamphour_out;

  double answer = 100 * (difference / milliamphour_batterycapacity);
  if (answer < 0)
  {
    // We have taken more out of the battery than put in, so must be zero SoC (or more likely out of calibration)
    return 0;
  }

  // Store result as fixed point decimal
  uint16_t SOC = 100 * answer;

  // Add a hard upper limit 999.99%
  if (SOC > 99999)
  {
    SOC = 99999;
  }

  return SOC;
}

void ConfigurePorts()
{
  // PA1 = SDA
  // PA2 = SCL
  // PA3 = UNUSED
  // PA4 = UNUSED
  // PA5 = RELAY OUTPUT
  // PA6 = RED LED
  // PA7 = GREEN LED

  // Set pins as Outputs (other pins are inputs)
  PORTA.DIR = GREENLED_PIN_BITMAP | REDLED_PIN_BITMAP | RELAY_PIN_BITMAP;

  // Relay off by default
  PORTA.OUTCLR = RELAY_PIN_BITMAP;

  // Set Port B digital outputs
  // PB0 = XDIR (RS485 transmit enable)
  // PB1 = ALERT
  // PB2 = TX (has to be set as output on tiny1614)
  // PB3 = RX
  // Set pins as Outputs (other pins are inputs)
  PORTB.DIR = PIN0_bm | PIN2_bm;

  PORTB.OUTSET = PIN2_bm; // TX is high

  USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm; // enable rx and tx
  // Enable interrupts
  sei();
}

void RedLED(bool value)
{
  if (value)
  {
    PORTA.OUTSET = REDLED_PIN_BITMAP;
  }
  else
  {
    PORTA.OUTCLR = REDLED_PIN_BITMAP;
  }
}

void GreenLED(bool value)
{
  if (value)
  {
    PORTA.OUTSET = GREENLED_PIN_BITMAP;
  }
  else
  {
    PORTA.OUTCLR = GREENLED_PIN_BITMAP;
  }
}

void EnableWatchdog()
{
  wdt_triggered = false;

  // Enter protection mode
  CCP = 0xD8;

  // 8 seconds
  WDT.CTRLA = WDT_PERIOD_enum::WDT_PERIOD_8KCLK_gc;

  wdt_reset();
}

/*
void DisableSerial0TX()
{
  //On tiny1614 this saves about 10mA of current
  USART0.CTRLB &= ~(USART_TXEN_bm); // Transmitter Enable bit mask.
}

void EnableSerial0TX()
{
  //When the transmitter is disabled, it will no longer override the TXD pin, and the pin
  //direction is automatically set as input by hardware, even if it was configured as output by the user
  //PB2 as OUTPUT
  PORTB.DIRSET = PIN2_bm;
  USART0.CTRLB |= USART_TXEN_bm; // Transmitter Enable bit mask.
}
*/

void WatchdogTriggered()
{
  // This is the watchdog timer - something went wrong and no serial activity received in over 8 seconds
  wdt_triggered = true;
  wdt_triggered_count++;
}

// pattern is a 32bit pattern to "play" on the RED LED to indicate failure
void __attribute__((noreturn)) blinkPattern(uint32_t pattern)
{

  // Show the error 4 times
  for (size_t x = 0; x < 4; x++)
  {
    uint32_t p = pattern;

    // Loop through the 32 bits - takes 1024ms in total
    for (size_t i = 0; i < 32; i++)
    {
      wdt_reset();
      if (p & 1)
      {
        RedLED(true);
      }
      else
      {
        RedLED(false);
      }
      p >>= 1;

      // Give user enough time to count the pulses
      delay(200);
    }

    // Switch off LED and wait half second before repeating
    RedLED(false);
    for (size_t i = 0; i < 40; i++)
    {
      wdt_reset();
      delay(50);
    }
  }

  // Both LEDs on whilst we wait for WDT
  RedLED(true);
  GreenLED(true);

  // Finally just hang - this will trigger the watchdog causing a reboot
  delay(10000);
}

// Sets SOC by setting "fake" in/out amphour counts
// value=8212 = 82.12%
void SetSOC(uint16_t value)
{
  // Assume battery is fully charged
  milliamphour_in = 1000 * (uint32_t)registers.batterycapacity_amphour;
  // And we have consumed this much...
  milliamphour_out = (1.0 - ((float)value / 10000.0)) * milliamphour_in;

  // Zero out readings using the offsets
  milliamphour_out_offset = milliamphour_out;
  milliamphour_in_offset = milliamphour_in;
}

void SaveConfig()
{
  WriteConfigToEEPROM((uint8_t *)&registers, sizeof(eeprom_regs));
}

bool SetRegister(uint16_t address, uint16_t value)
{
    static DoubleUnionType newvalue;

    switch (address)
    {
        case 4:
        case 6:
        //|40022|Fully charged voltage (4 byte double)
        case 21:
        //|40024|Tail current (Amps) (4 byte double)
        case 23:
        // Temperature limit (signed int16)
        case 29:
        // Bus Overvoltage (overvoltage protection)
        case 31:
        // BusUnderVolt
        case 33:
        // Shunt Over Voltage Limit (current limit)
        case 35:
        case 37:
        {
          // Set word[0] in preperation for the next register to be written
          newvalue.word[0] = value;
          break;
        }

        case 9:
        {
          // Bit flags
          setBitFlags(value);
          registers.bitflags = value;
          break;
        }

        // Allow reset of daily AH counters to ZERO
        case 12:
        case 13:
        {
          // Daily milliamphour_out (4 byte unsigned long uint32_t)
          daily_milliamphour_out = 0;
          break;
        }
        case 14:
        case 15:
        {
          // Daily milliamphour_in (4 byte  unsigned long uint32_t)
          daily_milliamphour_in = 0;
          break;
        }


      /* set not supported for ACS758
        case 18:
        {
          registers.shunt_max_current = value;
          CalculateLSB();
          break;
        }
        case 19:
        {
          // Register 40020
          registers.shunt_millivolt = value;
          CalculateLSB();
          break;
        }
      */


        case 20:
        {
          //|40021|Battery Capacity (ah)  (unsigned int16)
          registers.batterycapacity_amphour = value;
          break;
        }
        case 22:
        {
          //|40023|Fully charged voltage
          newvalue.word[1] = value;
          registers.fully_charged_voltage = newvalue.dblvalue;
          break;
        }

        case 24:
        {
          //|40025|Tail current (Amps)
          newvalue.word[1] = value;
          registers.tail_current_amps = newvalue.dblvalue;
          break;
        }
        case 25:
        {
          //|40026|Charge efficiency factor % (unsigned int16) (scale x100 eg. 10000 = 100.00%, 9561 = 95.61%)
          registers.charge_efficiency_factor = ((double)value) / 100.0;
          break;
        }
        case 26:
        {
          //|40027|State of charge % (unsigned int16) (scale x100 eg. 10000 = 100.00%, 8012 = 80.12%, 100 = 1.00%)
          // Change SOC by altering the amp hour counters
          SetSOC(value);
          break;
        }
        case 27:
        {
          // Register 40028
          // SHUNT_CAL register
          registers.R_SHUNT_CAL = value;
          break;
        }
        case 28:
        {
          // temperature limit
          // Case unsigned to int16 to cope with negative temperatures
          registers.temp_limit = value;
          break;
        }
        case 30:
        {
          // Bus Overvoltage (overvoltage protection).
          // Unsigned representation, positive value only. Conversion factor: 3.125 mV/LSB.
          // BusOverVolt.dblvalue = ((double)(uint16_t)i2c_readword(INA_REGISTER::BOVL)) * 0.003125F;
          newvalue.word[1] = value;
          registers.bus_overvoltage = newvalue.dblvalue;
          break;
        }
        case 32:
        {
          // Bus under voltage
          newvalue.word[1] = value;
          registers.bus_undervoltage = newvalue.dblvalue;
          break;
        }
        case 34:
        {
          // Shunt Over Voltage Limit (current limit)
          newvalue.word[1] = value;
          registers.bus_overcurrent = newvalue.dblvalue;

          break;
        }

        case 36:
        {
          // Shunt UNDER Voltage Limit (under current limit)
          newvalue.word[1] = value;
          registers.bus_undercurrent = newvalue.dblvalue;
          break;
        }

        case 38:
        {
          // Shunt Over POWER LIMIT
          newvalue.word[1] = value;
          registers.power_limit = newvalue.dblvalue;
          break;
        }

        case 39:
        {
          // Shunt Temperature Coefficient
          registers.R_SHUNT_TEMPCO = value;
          break;
        }

        case 45:
        {
          // Watchdog timer trigger count (like error counter)
          wdt_triggered_count = value;
          break;
        }

        default:
        {
          return false;
          break;
        }
    }

    // Rather than writing to EEPROM on every register change (there could be several)
    // mark the configuration as "dirty" and the loop() will write the config to EEPROM
    // in a few seconds time
    config_dirty = true;

    return true;
}

void setup()
{

  // Did we have a watchdog reboot?
  if (RSTCTRL.RSTFR & RSTCTRL_WDRF_bm)
  {
    // Must be first line of code
    WatchdogTriggered();
  }
  else
  {
    wdt_triggered_count = 0;
  }

  ConfigurePorts();
  RedLED(false);
  GreenLED(false);
  EnableWatchdog();


  if (ReadConfigFromEEPROM((uint8_t *)&registers, sizeof(eeprom_regs)) == false)
  {
    // Flash RED led 5 times to indicate facory reset
    for (size_t i = 0; i < 5; i++)
    {
      RedLED(true);
      delay(200);
      RedLED(false);
      delay(200);
    }

    // EEPROM is invalid, so apply "factory" defaults

    // Clear structure
    memset(&registers, 0, sizeof(eeprom_regs));

    // Defaults for battery capacity/voltages
    registers.batterycapacity_amphour = 280;
    registers.fully_charged_voltage = 3.50 * 16;
    registers.tail_current_amps = 20;
    registers.charge_efficiency_factor = 99.5;

    // ACS758 _hall_ sensor produces 2.5V at +200A, so 2.5V / 200A
    registers.shunt_max_current = 200;
    registers.shunt_millivolt = 2500;
    registers.RSHUNT = 2.5 / 200;

    // This is not enabled by default
    // The 16 bit register provides a resolution of 1ppm/°C/LSB
    registers.R_SHUNT_TEMPCO = 0;

    // Read the defaults from the INA228 chip as a starting point
    registers.bus_overcurrent = 100;
    registers.bus_undercurrent = -100;
    // 85volt max
    registers.bus_overvoltage = 57.6; // i2c_readword(INA_REGISTER::BOVL);
    registers.bus_undervoltage = 41.6;
    registers.temp_limit = 60.0;

    // Default Power limit = 5kW
    registers.power_limit = 5000.0;

    // By default, trigger relay on all alerts
    registers.relay_trigger_bitmap = ALL_ALERT_BITS;
  }

  setBitFlags(registers.bitflags);

  // Flash LED to indicate normal boot up
  for (size_t i = 0; i < 6; i++)
  {
    GreenLED(true);
    if (wdt_triggered)
    {
      RedLED(true);
    }
    delay(50);
    GreenLED(false);
    if (wdt_triggered)
    {
      RedLED(false);
    }
    delay(150);
  }

  // Disable RS485 receiver (debug!)
  #ifdef SERIALDEBUG
  PORTB.OUTSET = PIN0_bm;
  PORTB.PIN0CTRL = 0;
  #endif

  ads1115_setup();

  // Serial uses PB2/PB3 and PB0 for XDIR
  Serial.begin(ModBusBaudRate, MODBUSSERIALCONFIG);

  wdt_triggered = false;

  #ifndef SERIALDEBUG
  // 0x01= Enables RS-485 mode with control of an external line driver through a dedicated Transmit Enable (TE) pin.
  USART0.CTRLA |= USART_RS485_EXT_gc;
  modbus_configure(&Serial, ModBusBaudRate);
  #endif

/*
  // Default SOC% at 60%
  uint16_t soc = 6000;

  // We apply a "guestimate" to SoC based on voltage - not really accurate, but somewhere to start
  // only applicable to 24V/48V (16S) setups. These voltages should be the unloaded (no current flowing) voltage.
  // Assumption that its LIFEPO4 cells we are using
  double v = BusVoltage();
  // PP TODO: does not work yet, needs to be measured first


  if (v > 20 && v < 30)
  {
    // Scale up to use the 48V scale
    v = v * 2;
  }

  if (v > 40 && v < 60)
  {
    // 16S LIFEPO4...
    if (v >= 40.0)
      soc = 500;
    if (v >= 48.0)
      soc = 900;
    if (v >= 50.0)
      soc = 1400;
    if (v >= 51.2)
      soc = 1700;
    if (v >= 51.6)
      soc = 2000;
    if (v >= 52.0)
      soc = 3000;
    if (v >= 52.4)
      soc = 4000;
    if (v >= 52.8)
      soc = 7000;
    if (v >= 53.2)
      soc = 9000;
  }

  SetSOC(soc);
*/

  // Reset the daily counters
  daily_milliamphour_in = 0;
  daily_milliamphour_out = 0;
}


double TemperatureLimit()
{
  return registers.temp_limit;
}

// Calculated current output in Amperes.
// In the way this circuit is designed, NEGATIVE current indicates DISCHARGE of the battery
// POSITIVE current indicates CHARGE of the battery

ISR(PORTB_PORT_vect)
{
  uint8_t flags = PORTB.INTFLAGS;
  PORTB.INTFLAGS = flags; // clear flags

  if (flags && PIN1_bm)
  {
    RedLED(true);
    ads1115_isr();
  }
}

bool ReadHoldingRegister(uint16_t address, uint16_t *result)
{
    // Temporary variables to hold the register data
    static DoubleUnionType v;
    static DoubleUnionType c;
    static DoubleUnionType p;
    static DoubleUnionType shuntv;

    static DoubleUnionType BusOverVolt;
    static DoubleUnionType BusUnderVolt;
    static DoubleUnionType ShuntOverCurrentLimit;
    static DoubleUnionType ShuntUnderCurrentLimit;
    static DoubleUnionType PowerLimit;

    static DoubleUnionType copy_shunt_resistance;
    static DoubleUnionType copy_fully_charged_voltage;
    static DoubleUnionType copy_tail_current_amps;

    switch (address)
    {
        case 0:
        {
          // Voltage
          v.dblvalue = BusVoltage();
          *result =v.word[0];
          break;
        }

        case 1:
        {
          // Voltage
          *result =v.word[1];
          break;
        }

        case 2:
        {
          // Current
          c.dblvalue = Current();
          *result =c.word[0];
          break;
        }

        case 3:
        {
          // Current
          *result =c.word[1];
          break;
        }

        case 4:
        {
          // milliamphour_out
          *result =(uint16_t)((milliamphour_out - milliamphour_out_offset) >> 16);
          break;
        }

        case 5:
        {
          // milliamphour_out (low 16 bits)
          *result =(uint16_t)(milliamphour_out - milliamphour_out_offset);
          break;
        }

        case 6:
        {
          // milliamphour_in
          *result =(uint16_t)((milliamphour_in - milliamphour_in_offset) >> 16);
          break;
        }

        case 7:
        {
          // milliamphour_in (low 16 bits)
          *result =(uint16_t)(milliamphour_in - milliamphour_in_offset);
          break;
        }

        case 8:
        {
          // temperature
          *result =(int16_t)AttinyTemperature();
          break;
        }
        case 9:
        {
          // Various flags
          *result =bitFlags();
          break;
        }
        case 10:
        {
          // Power
          p.dblvalue = Power();
          *result =p.word[0];
          break;
        }
        case 11:
        {
          // Power
          *result =p.word[1];
          break;
        }
        case 12:
        {
          // daily milliamphour_out
          *result =(uint16_t)(daily_milliamphour_out >> 16);
          break;
        }
        case 13:
        {
          // daily milliamphour_out (low 16 bits)
          *result =(uint16_t)daily_milliamphour_out;
          break;
        }
        case 14:
        {
          // daily milliamphour_out
          *result =(uint16_t)(daily_milliamphour_in >> 16);
          break;
        }
        case 15:
        {
          // daily milliamphour_out (low 16 bits)
          *result =(uint16_t)daily_milliamphour_in;
          break;
        }
        case 16:
        {
          // Ohms to milliohm
          copy_shunt_resistance.dblvalue = 1000 * registers.RSHUNT;
          *result =copy_shunt_resistance.word[0];
          break;
        }
        case 17:
        {
          *result =copy_shunt_resistance.word[1];
          break;
        }

        case 18:
        {
          *result =registers.shunt_max_current;
          break;
        }
        case 19:
        {
          *result =registers.shunt_millivolt;
          break;
        }

        case 20:
        {
          //|40021|Battery Capacity (ah)  (unsigned int16)
          *result =registers.batterycapacity_amphour;
          break;
        }
        case 21:
        {
          //|40022|Fully charged voltage (4 byte double)
          copy_fully_charged_voltage.dblvalue = registers.fully_charged_voltage;
          *result =copy_fully_charged_voltage.word[0];
          break;
        }
        case 22:
        {
          //|40023|Fully charged voltage
          *result =copy_fully_charged_voltage.word[1];
          break;
        }
        case 23:
        {
          //|40024|Tail current (Amps) (4 byte double)
          copy_tail_current_amps.dblvalue = registers.tail_current_amps;
          *result =copy_tail_current_amps.word[0];
          break;
        }
        case 24:
        {
          //|40025|Tail current (Amps)
          *result =copy_tail_current_amps.word[1];
          break;
        }
        case 25:
        {
          //|40026|Charge efficiency factor % (unsigned int16) (scale x100 eg. 10000 = 100.00%, 9561 = 95.61%)
          *result =(uint16_t)(registers.charge_efficiency_factor * 100.0);
          break;
        }
        case 26:
        {
          //|40027|State of charge % (unsigned int16) (scale x100 eg. 10000 = 100.00%, 8012 = 80.12%, 100 = 1.00%)
          *result =CalculateSOC();
          break;
        }
        case 27:
        {
          // SHUNT_CAL register
          *result =registers.R_SHUNT_CAL;
          break;
        }
        case 28:
        {
          // temperature limit
          *result =(int16_t)TemperatureLimit();
          break;
        }
        case 29:
        {
          BusOverVolt.dblvalue = registers.bus_overvoltage;
          *result =BusOverVolt.word[0];
          break;
        }
        case 30:
        {
          *result =BusOverVolt.word[1];
          break;
        }
        case 31:
        {
          BusUnderVolt.dblvalue = registers.bus_undervoltage;
          *result =BusUnderVolt.word[0];
          break;
        }
        case 32:
        {
          *result =BusUnderVolt.word[1];
          break;
        }
        case 33:
        {
          ShuntOverCurrentLimit.dblvalue = registers.bus_overcurrent;

          *result =ShuntOverCurrentLimit.word[0];
          break;
        }
        case 34:
        {
          *result =ShuntOverCurrentLimit.word[1];
          break;
        }
        case 35:
        {
          ShuntUnderCurrentLimit.dblvalue = registers.bus_undercurrent;

          *result =ShuntUnderCurrentLimit.word[0];
          break;
        }
        case 36:
        {
          *result =ShuntUnderCurrentLimit.word[1];
          break;
        }

        case 37:
        {
          PowerLimit.dblvalue = registers.power_limit;
          *result =PowerLimit.word[0];
          break;
        }
        case 38:
        {
          *result =PowerLimit.word[1];
          break;
        }

        case 39:
        {
          // Shunt Temperature Coefficient
          *result =registers.R_SHUNT_TEMPCO;
          break;
        }

        case 40:
        {
          // INAXXX chip model number (should always be 0x0228)
          uint16_t dieid = 0x0190;
          *result =dieid;
          break;
        }

        // These settings would probably be better in a 0x2B function code
        // https://modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
        case 41:
        {
          // GITHUB version
          *result =GIT_VERSION_B1;
          break;
        }
        case 42:
        {
          // GITHUB version
          *result =GIT_VERSION_B2;
          break;
        }

        case 43:
        {
          // COMPILE_DATE_TIME_EPOCH
          uint32_t x = COMPILE_DATE_TIME_UTC_EPOCH >> 16;
          *result =(uint16_t)x;
          break;
        }
        case 44:
        {
          // COMPILE_DATE_TIME_EPOCH
          *result =(uint16_t)COMPILE_DATE_TIME_UTC_EPOCH;
          break;
        }
        case 45:
        {
          // Watchdog timer trigger count (like error counter)
          *result =wdt_triggered_count;
          break;
        }

        default:
          return false;

    } // end switch

    return true;
}

void loop()
{
    wdt_reset();

/*
    //Enable interrupts
    sei();
    //Start frame detection
    USART0.CTRLB |= USART_SFDEN_bm;

    //Switch off TX, saves current
    //DisableSerial0TX();

    if (!Serial.available())
    {
      //Enter sleep
      set_sleep_mode(SLEEP_MODE_STANDBY);
      sleep_enable();
      sleep_cpu();

      //Snoring can be heard at this point....
      sleep_disable();
    }
*/

    ads1115_loop();

    if (!haveAlert())
    {
      RedLED(false);
      PORTA.OUTCLR = RELAY_PIN_BITMAP;
    }
    else
    {
      // Turn relay on/off
      if (relayOn())
      {
        PORTA.OUTSET = RELAY_PIN_BITMAP;
      }
      else
      {
        PORTA.OUTCLR = RELAY_PIN_BITMAP;
      }
    }

  #ifndef SERIALDEBUG
  modbus_update();
  #endif


  if (millis() > timer)
  {
    if (config_dirty)
    {
      SaveConfig();
      config_dirty = false;
    }

    RedLED(true);

    // Do it again in X seconds
    timer = millis() + loop_delay_ms;

    double voltage = BusVoltage();
    double current;

    // Now to test if we need to reset SOC to 100% ?
    // Check if voltage is over the fully_charged_voltage and current UNDER tail_current_amps
    if (voltage > registers.fully_charged_voltage && (current=Current()) > 0 && current < registers.tail_current_amps)
    {
      // Battery has reached fully charged so wait for time counter
      soc_reset_counter++;

      // Test if counter has reached 3 minutes, indicating fully charge battery
      if (soc_reset_counter >= ((3 * 60) / (loop_delay_ms / 1000)))
      {
        // Now we reset the SOC, by clearing the registers, at this point SOC returns to 100%

        // This does have an annoying "feature" of clearing down todays Ah counts :-(
        // TODO: FIX THIS - probably need a set of shadow variables to hold the internal SOC and AH counts
        //                  but then when/how do we reset the Ah counts?

        max_soc_reset_counter = soc_reset_counter;
//        ResetChargeEnergyRegisters();
        last_charge_coulombs = 0;
        soc_reset_counter = 0;
        SetSOC(10000);
      }
    }
    else
    {
      // Voltage or current is out side of monitoring limits, so reset timer count
      soc_reset_counter = 0;
    }

    if (!haveAlert())
    {
      // Turn LED off if alert is not active
      RedLED(false);
    } // end if

  } // end if
}