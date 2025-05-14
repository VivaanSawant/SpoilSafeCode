#ifndef BYTEARRAY_H
#define BYTEARRAY_H

#include <Arduino.h>
#include <Wire.h>

class bytearray {
public:
    bytearray(int length);              // create a bytearray of a specified length
    bytearray(int bytes[], int length); // create a bytearray from an array of ints
    bytearray(String str);              // create a bytearray from a String

    byte& operator[](int index);        // allows for accessing elements through the [] operator
    bytearray operator+(const bytearray &bytes);    // concatenates bytearrays
    bytearray& operator+=(const bytearray &bytes);  // concatenates and overwrites bytearrays

    void print();                       // prints the bytearray for debugging
    int length();                       // get the length of the bytearray
    String toString();                  // convert a bytearray to a String

private:
    int len;
    byte arr[I2C_BUFFER_LENGTH] = {};
    void err(const char *fn, String msg);      // prints error message
};

#endif