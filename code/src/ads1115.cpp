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

#include <util/atomic.h>
#include <ADS1115_lite.h>
#include "ads1115.h"
#include "main.h"




uint32_t lastVoltage_mV = 0;
int32_t lastCurrent_mA = 0;

uint32_t timeLastCurrentMeasurement = 0;

uint32_t lastCurrentIntegrationMillis = 0;
int32_t integrated_mA_ms = 0;

struct bitflags current_bitflags;

extern uint32_t milliamphour_out_lifetime;
extern uint32_t milliamphour_in_lifetime;

extern uint32_t daily_milliamphour_out;
extern uint32_t daily_milliamphour_in;

extern uint32_t milliamphour_out;
extern uint32_t milliamphour_in;

extern uint32_t milliamphour_out_offset;
extern uint32_t milliamphour_in_offset;

extern eeprom_regs registers;

extern void __attribute__((noreturn)) blinkPattern(uint32_t pattern);

#define ADS1115_MODE_VOLTAGE 1
#define ADS1115_MODE_CURRENT 2
/**
 * The sampling mode (current/voltage) we just set. If not the same as ads1115_mode the next result at ISR is thrown away and ads1115_mode is set
 */
uint8_t ads1115_pending_mode = 0;

/**
 * The actual sampling mode
 */
uint8_t ads1115_mode = 0;

volatile bool ads1115_rdy = false;

ADS1115_lite ads;


bool i2c_write16bitRegister(const uint8_t address, const uint8_t inareg, const uint16_t data)
{
  Wire.beginTransmission(address);
  Wire.write(inareg);
  Wire.write((uint8_t)(data >> 8)); // Write the first (MSB) byte
  Wire.write((uint8_t)data);        // and then the second byte
  uint8_t result = Wire.endTransmission();

  // Delay after making a write to INA chip
  delayMicroseconds(10);

  return result == 0;
}


uint32_t ads1115_setup()
{
    // for internal temperature measurement
    // https://onlinedocs.microchip.com/pr/GUID-C541EA24-5EC3-41E5-9648-79068F9853C0-en-US-3/index.html?GUID-C39DBA19-2081-4EF2-9F86-F64DFC4B4442
    VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
    ADC0.CTRLC = ADC_REFSEL_INTREF_gc;
    ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
    ADC0.CTRLD = ADC_INITDLY_DLY32_gc;
    ADC0.SAMPCTRL = ADC_ACC32;


    Wire.begin();
    // Change TWI pins to use PA1/PA2 and not PB1/PB0
    Wire.swap(1);
    // Use fast i2c
    Wire.setClock(400000);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // ALARM PIN
        PORTB.DIRCLR = PIN1_bm;
        PORTB.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_enum::PORT_ISC_FALLING_gc;
    }

    ads1115_rdy = false;

    memset( &current_bitflags, 0, sizeof(current_bitflags) );

    // See if the device is connected/soldered on board
    Wire.beginTransmission(ADS1115address); // transmit to device
    if (Wire.endTransmission() > 0)
    {
        blinkPattern(err_INA228Missing);
        return err_INA228Missing;
    }

    ads = ADS1115_lite(ADS1115address);
    if( !ads.testConnection() )
    {
        blinkPattern(err_INA228Reset);
        return err_INA228Reset;
    }
}

/**
 * set ADS1115 to continuous Ampere sampling
 * 
 * ACS758 - 200B on AIN2; Vcc/2 on AIN3
 * * ACS758 - 200B output with 0 Amps = Vcc/2 = 2.5V | +200 Amps = Vcc | -200 Amps = 0V
 * 
 * This means that it makes sense to sample the differential between AIN2 and AIN3 (Vcc/2)
 * --> so with +200A we have 2.5V differential
 * --> with 0A we have 0V differential
 * --> with -200A we have -2.5V differential
*/
bool ads1115_setupContinuousAmpereSampling()
{
    // differential AIN2(input)<->AIN3(Vcc)
    ads.setMux(ADS1115_REG_CONFIG_MUX_DIFF_2_3);
    ads.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS);
    ads.setGain(ADS1115_REG_CONFIG_PGA_6_144V);
    if( !ads.triggerContinuous() )
    {
        ads1115_pending_mode = ADS1115_MODE_CURRENT;
        return true;
    }
    return false;
}


/**
 * set ADS1115 to continuous Ampere sampling
 * 
 * ACPL-C870 isolates & converts a VIn 1:1 to a differential between AIN0<->AIN1:
 * * max differential = 2.0V (max values of ACPL)
 * * min differential = 0.0V
 * 
 * Voltage calculation is using a Voltage divider of (ideally) 2.010 MOhm ./. 4k7 Ohm
 * ---> Vout = Vin * 4700 / (2010000 + 4700)
 * ---> Vin = Vout / (4700 / (2010000 + 4700))
 * ---> Vin = Vout * (2010000 + 4700) / 4700
 * ---> 850V ^= 1.98292V
*/
bool ads1115_setupVoltageSampling()
{
    // differential AIN2<->AIN3
    ads.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
    ads.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS);
    ads.setGain(ADS1115_REG_CONFIG_PGA_2_048V);
    if( !ads.triggerContinuous() )
    {
        ads1115_pending_mode = ADS1115_MODE_VOLTAGE;
        return true;
    }
    return false;
}

void ads1115_isr()
{
    ads1115_rdy = true;
}


void ads1115_loop()
{
    if( !ads1115_rdy )
    {
        return;
    }
    ads1115_rdy = false;

    int16_t res = ads.getConversionResult();
    if( ads1115_pending_mode != ads1115_mode )
    {
        // throw away one result
        ads1115_mode = ads1115_pending_mode;
        return;
    }

    uint32_t m = millis();

    if( ads1115_mode == ADS1115_MODE_CURRENT )
    {
/*
 * ACS758 - 200B on AIN2; Vcc/2 on AIN3
 * * ACS758 - 200B output with 0 Amps = Vcc/2 = 2.5V | +200 Amps = Vcc | -200 Amps = 0V
 * 
 * This means that it makes sense to sample the differential between AIN2 and AIN3 (Vcc/2)
 * --> so with +200A we have 2.5V differential
 * --> with 0A we have 0V differential
 * --> with -200A we have -2.5V differential
*/
        int32_t ads_mV = 6144 * res / 32767;
        lastCurrent_mA = ads_mV 
          * 80;  // (200000mA / 2500 mV)

        // this does even work in case of millis overrun
        // Why? see https://arduino.stackexchange.com/a/12588 
        // in https://arduino.stackexchange.com/questions/12587/how-can-i-handle-the-millis-rollover 
        // for a length explanation
        uint32_t timeSinceLast = m - timeLastCurrentMeasurement;
        timeLastCurrentMeasurement = m;

        // first integrate to uAs (mA*ms) and below every second into mAh
        // max is +/- 0x0BEBC200 = 128 * (7.8125ms * +/-200000mA) = 1000ms * +/-200000mA
        integrated_mA_ms += lastCurrent_mA * timeSinceLast;  

        double A = Current();
        current_bitflags.ocurr = A >= registers.bus_overcurrent ? 1 : 0;
        current_bitflags.ucurr = A <= registers.bus_undercurrent ? 1 : 0;
    }
    else if( ads1115_mode == ADS1115_MODE_VOLTAGE )
    {
        uint32_t ads_mV = 2048 * (uint32_t)max(0,res) / 32767;  // voltage cannot be negative and should not be swapped
        lastVoltage_mV = ads_mV * (2010000+4700) / 4700;   // this barely fits into uint32_t as max would be 4126105600 or 0xF5EF6000

        double V = BusVoltage();
        current_bitflags.busol = V >= registers.bus_overvoltage ? 1 : 0;
        current_bitflags.busul = V <= registers.bus_undervoltage ? 1 : 0;
    }
    else
    {
        return;
    }

    if( (m-lastCurrentIntegrationMillis) >= 1000 )
    {
        // make mAh (milliamp HOURS) out of uAs (millamp milliseconds)
        integrated_mA_ms /= 3600000;
        if( integrated_mA_ms > 0 )
        {
            milliamphour_out += integrated_mA_ms;
            milliamphour_out_lifetime += integrated_mA_ms;
            daily_milliamphour_out += integrated_mA_ms;
        }
        else
        {
            integrated_mA_ms *= -1;
            milliamphour_in += integrated_mA_ms;
            milliamphour_in_lifetime += integrated_mA_ms;
            daily_milliamphour_in += integrated_mA_ms;
        }

        current_bitflags.pol = Power() > registers.power_limit ? 1 : 0;

        double t = AttinyTemperature();
        current_bitflags.tmpol = t > registers.temp_limit ? 1 : 0;

        lastCurrentIntegrationMillis = m;
    }
}


double BusVoltage()
{
  return (double)lastVoltage_mV / (double)1000.0;
}


double Current()
{
  return (double)lastCurrent_mA / (double)1000.0;
}

// Calculated power output.  Output value in watts. Unsigned representation. Positive value.
double Power()
{
  return BusVoltage() * Current();
}


// read Attiny temp sensor
// see https://onlinedocs.microchip.com/pr/GUID-C541EA24-5EC3-41E5-9648-79068F9853C0-en-US-3/index.html?GUID-C39DBA19-2081-4EF2-9F86-F64DFC4B4442
double AttinyTemperature()
{
  int8_t sigrow_offset = SIGROW.TEMPSENSE1;  // Read signed value from signature row
  uint8_t sigrow_gain = SIGROW.TEMPSENSE0;    // Read unsigned value from signature row
  uint16_t adc_reading = 0;   // ADC conversion result with 1.1 V internal reference 

  uint32_t temp = adc_reading - sigrow_offset;
  temp *= sigrow_gain;  // Result might overflow 16 bit variable (10bit+8bit)
  temp += 0x80;               // Add 1/2 to get correct rounding on division below
  temp >>= 8;                 // Divide result to get Kelvin 
  double dietemp = (double)temp + (double)273.15;
  return dietemp;
}


uint16_t bitFlags()
{
    return *((uint16_t*)&current_bitflags);
}

void setBitFlags(uint16_t bitflags)
{
    *((uint16_t*)&current_bitflags) = bitflags & SETTABLE_BITFLAGS_MASK;
}

bool haveAlert()
{
    return bitFlags() & RELAY_BITS;
}

bool relayOn()
{
    bool relay_state = (
        current_bitflags.relay_trigger_busol && current_bitflags.busol ||
        current_bitflags.relay_trigger_busul && current_bitflags.busul ||
        current_bitflags.relay_trigger_pol && current_bitflags.pol ||
        current_bitflags.relay_trigger_ocurr && current_bitflags.ocurr ||
        current_bitflags.relay_trigger_ucurr && current_bitflags.ucurr ||
        current_bitflags.relay_trigger_tmpol && current_bitflags.tmpol
    );
    return relay_state;
}