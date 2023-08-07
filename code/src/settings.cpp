/*

EEPROM Settings storage with checksum

(c)2023 Patrick Prasse

based on https://github.com/stuartpittaway/diyBMS-CurrentShunt
(c)2021 Stuart Pittaway

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

#include "settings.h"


void WriteConfigToEEPROM(uint8_t *settings, uint16_t size)
{
  //TODO: We should probably check EEPROM.length() to ensure its big enough

  //Generate and save the checksum for the setting data block
  
  uint16_t checksum = CRC16.modbus(settings,size);

  uint16_t existingChecksum;
  EEPROM.get(0, existingChecksum);

  if (checksum == existingChecksum && checksum!=0)
  {
    //nothing has changed, so just return
    return;
  }

  //Start write at address 2, checksum is bytes 0 and 1
  uint16_t EEPROMaddress = 2;
  for (uint16_t i = 0; i < size; i++)
  {
    EEPROM.update(EEPROMaddress, settings[i]);
    EEPROMaddress++;
  }

  EEPROM.put(0, checksum);
}

bool ReadConfigFromEEPROM(uint8_t *settings, uint16_t size)
{
  uint16_t EEPROMaddress = 2;
  for (uint16_t i = 0; i < size; i++)
  {
    settings[i] = EEPROM.read(EEPROMaddress);
    EEPROMaddress++;
  }

  // Calculate the checksum
  uint16_t checksum = CRC16.modbus(settings,size);

  uint16_t existingChecksum;
  EEPROM.get(0, existingChecksum);

  if (checksum == existingChecksum)
  {
    //Return TRUE
    return true;
  }

  //Original data is now corrupt so return FALSE
  return false;
}

void FactoryDefault(uint16_t size)
{
  //Clear checksum
  EEPROM.put(0, 0xFFFF);
}
