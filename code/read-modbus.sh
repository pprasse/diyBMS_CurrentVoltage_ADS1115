#!/bin/bash
#pip install modbus-cli
modbus -b 9600 -s 90 --byte-order=mixed -r modbus_registers /dev/ttyUSB1 \*
modbus -b 9600 -s 90 -r modbus_registers_int32 /dev/ttyUSB1 \*

