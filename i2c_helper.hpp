#ifndef I2C_H
#define I2C_H

#include <Arduino.h>
#include <Wire.h>
#include "bytearray.hpp"

bytearray i2c_readFrom(byte address, byte len);

void i2c_writeTo(byte address, bytearray data);

#endif