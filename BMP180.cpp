
/*
 *  Bosch Sensortec, BMP180 Digital pressure sensor
 *
 *  BMP180.cpp
 *
 *  Creado: 03 Jun 2017 por jgarcia1112@gmail.com
 *  Villahermosa, Tabasco. Mexico
 *
 *  Modificado: 16 Jun 2017 por Javier Garcia.
 *
 *  Version 1.0 - Este codigo es de dominio publico y sin ninguna garantia
 *
 *
 */


#include "BMP180.h"
#include <math.h>

/**
 * Default constructor, uses default I2C device address.
 * @see BMP180_DEFAULT_ADDRESS
 */
BMP180::BMP180() {
    devAddr = BMP180_DEFAULT_ADDRESS;
}

/**
 * Specific address constructor.
 * @param address Specific device address
 * @see BMP180_DEFAULT_ADDRESS
 */
BMP180::BMP180(uint8_t address) {
    devAddr = address;
}

/**
 * Prepare device for normal usage.
 * @see BMP180_MODE_ULTRAHIGHRES
 */
void BMP180::begin(uint8_t mode) {
    measureMode = mode;
    // load sensor's calibration constants
    loadCalibration();
}

/**
 * Verify the device is connected and available.
 */
bool BMP180::testConnection() { 
    readBytes(BMP180_RA_AC1_H);
    return Wire_read() == 1;
}

void BMP180::readBytes(uint8_t reg, uint8_t qtty) {
    Wire.beginTransmission(devAddr);                    //Start communicating with the BMP-180
    Wire_write(reg);                                    //Send the requested starting register
    Wire.endTransmission();                             //End the transmission
    Wire.requestFrom(devAddr, qtty);                    //Request "n" bytes from the BMP-180
    while(Wire.available() < qtty);                     //Wait until all the bytes are received
}

void BMP180::writeByte(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(devAddr);                    //Start communicating with the BMP-180
    Wire_write(reg);                                    //Send the requested starting register
    Wire_write(value);                                  //Send the value register
    Wire.endTransmission();                             //End the transmission
}

/* calibration register methods */

void BMP180::loadCalibration() {
    readBytes(BMP180_RA_AC1_H, 22);
    ac1 = ((int16_t)Wire_read()  << 8) | Wire_read();    //Add the low and high byte to the variable
    ac2 = ((int16_t)Wire_read()  << 8) | Wire_read();
    ac3 = ((int16_t)Wire_read()  << 8) | Wire_read();
    ac4 = ((uint16_t)Wire_read() << 8) | Wire_read();
    ac5 = ((uint16_t)Wire_read() << 8) | Wire_read();
    ac6 = ((uint16_t)Wire_read() << 8) | Wire_read();
    b1 =  ((int16_t)Wire_read()  << 8) | Wire_read();
    b2 =  ((int16_t)Wire_read()  << 8) | Wire_read();
    mb =  ((int16_t)Wire_read()  << 8) | Wire_read();
    mc =  ((int16_t)Wire_read()  << 8) | Wire_read();
    md =  ((int16_t)Wire_read()  << 8) | Wire_read();
}

uint16_t BMP180::getRawTemperature() {
    // Send control to start convertion
    writeByte(BMP180_RA_CONTROL, BMP180_MODE_TEMPERATURE);

    // Wait to finish convertion
    do {
        readBytes(BMP180_RA_CONTROL);
    } while ((Wire_read() & BMP180_RA_SCO));

    // Read raw value
    readBytes(BMP180_RA_MSB, 2);
    return ((uint16_t)Wire_read() << 8) | Wire_read(); 
}

uint32_t BMP180::getRawPressure() {
    // Send control to start convertion
    writeByte(BMP180_RA_CONTROL, measureMode);
    
    // Wait to finish convertion
    do {
        readBytes(BMP180_RA_CONTROL);
    } while ((Wire_read() & BMP180_RA_SCO));
    
    // Read raw value
    readBytes(BMP180_RA_MSB, 3);
    return (((uint32_t)Wire_read() << 16) | ((uint16_t)Wire_read() << 8) | Wire_read()) >> (8 - ((measureMode & 0xC0) >> 6));
}

float BMP180::getTemperatureC() {
    /*
     Datasheet formula:
     UT = raw temperature
     X1 = (UT - AC6) * AC5 / 2^15
     X2 = MC * 2^11 / (X1 + MD)
     B5 = X1 + X2
     T = (B5 + 8) / 2^4
     */
    int32_t ut = getRawTemperature();
    int32_t x1 = ((ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
    int32_t x2 = ((int32_t)mc << 11) / (x1 + md);
    b5 = x1 + x2;
    return (float)((b5 + 8) >> 4) / 10.0f;
}

float BMP180::getTemperatureF() {
    return getTemperatureC() * 9.0f / 5.0f + 32;
}

float BMP180::getPressure() {
    /*
     Datasheet forumla
     UP = raw pressure
     B6 = B5 - 4000
     X1 = (B2 * (B6 * B6 / 2^12)) / 2^11
     X2 = AC2 * B6 / 2^11
     X3 = X1 + X2
     B3 = ((AC1 * 4 + X3) << oss + 2) / 4
     X1 = AC3 * B6 / 2^13
     X2 = (B1 * (B6 * B6 / 2^12)) / 2^16
     X3 = ((X1 + X2) + 2) / 2^2
     B4 = AC4 * (unsigned long)(X3 + 32768) / 2^15
     B7 = ((unsigned long)UP - B3) * (50000 >> oss)
     if (B7 < 0x80000000) { p = (B7 * 2) / B4 }
     else { p = (B7 / B4) * 2 }
     X1 = (p / 2^8) * (p / 2^8)
     X1 = (X1 * 3038) / 2^16
     X2 = (-7357 * p) / 2^16
     p = p + (X1 + X2 + 3791) / 2^4
     */
    uint32_t up = getRawPressure();
    uint8_t oss = (measureMode & 0xC0) >> 6;
    int32_t p;
    int32_t b6 = b5 - 4000;
    int32_t x1 = ((int32_t)b2 * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = ((int32_t)ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((int32_t)ac1 * 4 + x3) << oss) + 2) >> 2;
    x1 = ((int32_t)ac3 * b6) >> 13;
    x2 = ((int32_t)b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = ((uint32_t)ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)up - b3) * (uint32_t)(50000UL >> oss);
    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    return p + ((x1 + x2 + (int32_t)3791) >> 4);
}

float BMP180::getAltitude(float pressure, float seaLevelPressure) {
    return 44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

float BMP180::getSeaLevelPressure(float pressure, float altitude) {
    return pressure / pow(1.0 - (altitude/44330.0), 5.255);
}







