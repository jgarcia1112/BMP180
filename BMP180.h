
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


#ifndef _BMP180_H_
#define _BMP180_H_

#if ARDUINO >= 100
    #include "Arduino.h"
    #define Wire_write  Wire.write
    #define Wire_read   Wire.read
#else
    #include "WProgram.h"
    #define Wire_write  Wire.send
    #define Wire_read   Wire.receive
#endif

#include <Wire.h>

#define BMP180_ADDRESS              0x77
#define BMP180_DEFAULT_ADDRESS      BMP180_ADDRESS
#define BMP180_RA_AC1_H     0xAA    /* AC1_H */
#define BMP180_RA_AC2_H     0xAC    /* AC2_H */
#define BMP180_RA_AC3_H     0xAE    /* AC3_H */
#define BMP180_RA_AC4_H     0xB0    /* AC4_H */
#define BMP180_RA_AC5_H     0xB2    /* AC5_H */
#define BMP180_RA_AC6_H     0xB4    /* AC6_H */
#define BMP180_RA_B1_H      0xB6    /* B1_H */
#define BMP180_RA_B2_H      0xB8    /* B2_H */
#define BMP180_RA_MB_H      0xBA    /* MB_H */
#define BMP180_RA_MC_H      0xBC    /* MC_H */
#define BMP180_RA_MD_H      0xBE    /* MD_H */
#define BMP180_RA_CONTROL   0xF4    /* CONTROL */
#define BMP180_RA_MSB       0xF6    /* MSB */
#define BMP180_RA_LSB       0xF7    /* LSB */
#define BMP180_RA_XLSB      0xF8    /* XLSB */
#define BMP180_RA_SCO       0x20    /* Start of conversion */

#define BMP180_RA_SOFT_RESET        0xE0
#define BMP180_RA_CHIP_ID           0xD0    /* for BMP180 returns: 0x77 */

#define BMP180_MODE_TEMPERATURE     0x2E
#define BMP180_MODE_ULTRALOWPOWER   0x34
#define BMP180_MODE_STANDARD        0x74
#define BMP180_MODE_HIGHRES         0xB4
#define BMP180_MODE_ULTRAHIGHRES    0xF4

class BMP180 {
public:
    BMP180();
    BMP180(uint8_t address);

    void begin(uint8_t mode = BMP180_MODE_ULTRAHIGHRES);
    bool testConnection();

    void        loadCalibration();
    uint16_t    getRawTemperature();
    float       getTemperatureC();
    float       getTemperatureF();
    uint32_t    getRawPressure();
    float       getPressure();
    float       getAltitude(float pressure, float seaLevelPressure=101325);
    float       getSeaLevelPressure(float pressure, float altitude);

private:
    uint8_t devAddr;

    void        readBytes(uint8_t reg, uint8_t qtty = 1);
    void        writeByte(uint8_t reg, uint8_t value);

    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
    int32_t b5;
    uint8_t measureMode;
};

#endif /* _BMP180_H_ */








