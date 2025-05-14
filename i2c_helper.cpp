#include <Arduino.h>
#include <Wire.h>
#include "bytearray.hpp"

#include "i2c_helper.hpp"

/**
 * Reads n bytes from the i2c bus and returns them as a bytearray
 * @param address   Address to request data from
 * @param n         Number of bytes to read
 * @return          Data from i2c in a bytearray
 */
bytearray i2c_readFrom(byte address, byte n) {
    bytearray data(n);

    Wire.requestFrom(address, n);
    for (int i = 0; i < n; i++) { 
        data[i] = Wire.read(); 
    }
    return data;
}

/**
 * Writes a bytearray to an address on the i2c bus
 * @param address   Address to write data to
 * @param data      Data to write as a bytearray
 */
void i2c_writeTo(byte address, bytearray data) {
    Wire.beginTransmission(address);
    for (int i = 0; i < data.length(); i++) { 
        Wire.write(data[i]); 
    }
    Wire.endTransmission();
}