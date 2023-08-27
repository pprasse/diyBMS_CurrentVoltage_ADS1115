# diyBMS_CurrentVoltage_ADS1115

Isolated current and isolated voltage sensor for up to 850V & 200A based on ADS1115 and attiny1614 with isolated RS485 / MODBUS. (based on https://github.com/stuartpittaway/diyBMS-CurrentShunt)

# Current / Voltage measurement

The current measurement uses an ACS758ECB-200B hall-effect based current sensor IC.

The voltage measurement is isolated using an ACPL-C870 Precision Optically Isolated Voltage Sensor.

The analog signal from both measurements are converted using a ADS1115 16 bit ADC.

Coulomb calculation takes place in using an attiny1614 and communication uses isolated RS485 / MODBUS.

# Basis

The project is based on https://github.com/stuartpittaway/diyBMS-CurrentShunt and was created to allow using high voltage batteries for diyBMSv4.

# BOARD

The schematics and board are in the directory KiCad.

# CODE / FIRMWARE

The firmware in the directory code is losely based on Stuard Pittaway's diyBMS CurrentShunt. It uses the same MODBUS RTU registers.

## Coulomb calculation

The charge that goes in and out is calculated by sampling the current 128 times per second using the ADS1115 and measuring the time beween each sample. These are then integrated first to mA*ms (uAs) and then every second to mAh (milliamphours).

## Voltage measurement

The voltage is measured every 1 second as it's irrelevant for charge calculation but interesting for power calculation and over/undervoltage checks.

## MODBUS registers

See [MODBUS Registers.md](./code/MODBUS%20Registers.md)

Maximum compatibililty to diyBMS-CurrentShunt should be maintained to allow using the diyBMS4 ESP32 code unmodified.

Please note that due to compatibility the double registers are in mixed endian (floats compromised of 2 16bit registers in little endian, but individual 16 bit registers are big endian) BUT the 32bit integers are big endian.

Use [read-modbus.sh](./code/read-modbus.sh) ```# code/read-modbus.sh``` to read the modbus registers.


# WARNING

This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life.  

This board was designed to work with high DC voltages which are notoriously dangerous to work with and are a risk to life. Please make sure you have the appropriate knowledge and always use fuses specified for DC.

There is no warranty, it may not work as expected or at all.

The use of this project is done so entirely at your own risk.  It may involve electrical voltages which could kill - if in doubt, seek help.

The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.


# License

This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales License.

https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

You are free to:
* Share — copy and redistribute the material in any medium or format
* Adapt — remix, transform, and build upon the material
The licensor cannot revoke these freedoms as long as you follow the license terms.

Under the following terms:
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* Non-Commercial — You may not use the material for commercial purposes.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.


Notices:
You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation.

No warranties are given. The license may not give you all of the permissions necessary for your intended use. For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.
