#!/bin/bash
#pip install modbus-cli
modbus -b 9600 -s 90 --byte-order=mixed -r modbus_registers /dev/ttyUSB1 \*

