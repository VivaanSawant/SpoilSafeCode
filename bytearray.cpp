#include "bytearray.hpp"
#include <Arduino.h>
#include <Wire.h>

#define printerr(msg) err(__FUNCTION__, msg)

/**
 * Constructor, creates a blank bytearray of specified length
 * @param[in] length Length of the bytearray
 */
bytearray::bytearray(int length) {
    if(!(0 < length && length <= I2C_BUFFER_LENGTH)) {
        String errmsg("attempt to create bytearray of length ");
        errmsg += length;
        errmsg += '\n';
        errmsg += "length must <= ";
        errmsg += I2C_BUFFER_LENGTH;
        printerr(errmsg);
    }
    this->len = length;
}

/**
 * Constructor, creates a bytearray from an int array
 * @param[in] int_arr[length] Int array to create the bytearray from, max length 32
 * @param[in] length Length of the bytearray, must not exceed length of the int array
 */
bytearray::bytearray(int int_arr[], int length) {
    if(!(0 < length && length <= I2C_BUFFER_LENGTH)) {
        String errmsg("(from int array) attempt to create bytearray of length ");
        errmsg += length;
        errmsg += '\n';
        errmsg += "length must <= ";
        errmsg += I2C_BUFFER_LENGTH;
        printerr(errmsg);
    }

    for (int i = 0; i < length; i++) {
        if(!(0 <= int_arr[i] && int_arr[i] <= 0xFF)) {
            String errmsg("(from int array) attempt to create bytearray with out-of-range value\n");
            errmsg += String("value ") + String(int_arr[i]) + String(" at index ") + String(i) + String('\n');
            errmsg += "value must be between 0 and 255 (0xFF) inclusive";
            printerr(errmsg);
        }
        this->arr[i] = (byte)int_arr[i];
    }

    this->len = length;
}

/**
 * Constructor, creates a bytearray from an Arduino String
 * @param[in] str Arduino String to create the bytearray from, max length 31
 */
bytearray::bytearray(String str) {
    if(!(0 < str.length() && str.length() <= I2C_BUFFER_LENGTH - 1)) {
        String errmsg("(from String) attempt to create bytearray of length ");
        errmsg += str.length();
        errmsg += '\n';
        errmsg += "length must <= ";
        errmsg += I2C_BUFFER_LENGTH - 1;
        printerr(errmsg);
    }

    this->len = str.length();
    str.toCharArray((char*)(this->arr), this->len + 1);
}

/**
 * @brief Array subscript operator [] with index checking
 */
byte& bytearray::operator[](int index) {
    if(!(0 <= index && index < this->len)) {
        String errmsg("attempt to access value at index " + index);
        errmsg += '\n';
        errmsg += "index must < ";
        errmsg += this->len;
        printerr(errmsg);
    }
    return this->arr[index];
}

/** 
  * @brief Concatenates bytearrays
  * @param[in] bytes Bytearray to concatenate with
  * @returns A new bytearray
  */
bytearray bytearray::operator+(const bytearray &bytes) {
    if(!(this->len + bytes.len <= I2C_BUFFER_LENGTH)) {
        String errmsg("attempt to concatenate bytearray with total length ");
        errmsg += this->len + bytes.len;
        errmsg += '\n';
        errmsg += "total length must <= ";
        errmsg += I2C_BUFFER_LENGTH;
        printerr(errmsg);
    }

    bytearray result(this->len + bytes.len);

    for (int i = 0; i < this->len; i++) {
        result.arr[i] = this->arr[i];
    }
    for (int i = 0; i < bytes.len; i++) {
        result.arr[this->len + i] = bytes.arr[i];
    }

    return result;
}

/** 
  * @brief Concatenates bytearrays
  * @param[in] bytes Bytearray to concatenate with
  * @returns Overwrites the original bytearray
  */
bytearray& bytearray::operator+=(const bytearray &bytes) {
    if(!(len + bytes.len <= I2C_BUFFER_LENGTH)) {
        String errmsg("attempt to concatenate bytearray with total length ");
        errmsg += len + bytes.len;
        errmsg += '\n';
        errmsg += "total length must <= ";
        errmsg += I2C_BUFFER_LENGTH;
        printerr(errmsg);
    }

    for (int i = 0; i < bytes.len; i++) {
        this->arr[len++] = bytes.arr[i];
    }

    return *this;
}

/**
 * @brief Prints the bytearray for debugging
 */
void bytearray::print() {
  Serial.print("{ ");
  for (int i = 0; i < this->len - 1; i++) {
    Serial.print(this->arr[i]);
    Serial.print(", ");
  }
  Serial.print(this->arr[this->len - 1]);
  Serial.println(" }");
}

/**
 * @brief Returns length of the bytearray
 */
int bytearray::length() {
    return (int)(this->len);
}

/**
 * @brief Returns the bytearray as an Arduino String
 */
String bytearray::toString() {
    String str = String();
    for (int i = 0; i < len; i++) {
        str += (char)((this->arr)[i]);
    }
    return str;
}

/**
 * Prints an error message
 * @param fn function name
 * @param msg error message
 */
void bytearray::err(const char *fn, String msg) {
        Serial.print(fn);
        Serial.print(": ");
        Serial.println(msg);
        delay(100);
        abort();
}