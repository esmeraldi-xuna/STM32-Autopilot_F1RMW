	

/**
 * @brief       BMP085.h
 * @details     Digital pressure sensor.
 *              Header file.
 *
 *
 * @return      NA
 *
 * @author      Manuel Caballero
 * @date        25/August/2017
 * @version     25/August/2017    The ORIGIN
 * @pre         NaN.
 * @warning     NaN
 * @pre         This code belongs to AqueronteBlog ( http://unbarquero.blogspot.com ).
 */
#ifndef BMP085_H
#define BMP085_H
 
#include "mbed.h"
 
 
/**
    Example:
 
#include "mbed.h"
#include "BMP085.h"
 
BMP085 myBarometricSensor  ( I2C_SDA, I2C_SCL, BMP085::BMP085_ADDRESS, 400000 );
Serial pc                  ( USBTX, USBRX );                                                 // tx, rx
 
 
Ticker serial;
 
DigitalOut myled(LED1);
 
BMP085::Vector_cal_coeff_t          myCalCoeff;
BMP085::Vector_temp_f               myUT;
BMP085::Vector_pressure_f           myUP;
BMP085::Vector_compensated_data_f   myTrueData;
 
uint32_t myState = 0;
 
void sendDATA ( void )
{
    switch ( myState ) {
        case 0:
        // Trigger a new temperature measurement
            myBarometricSensor.BMP085_TriggerTemperature ();
            myState = 1;
            break;
 
        case 1:
        // Read the uncompensated temperature data and trigger a new pressure measurement
            myBarometricSensor.BMP085_ReadRawTemperature ( &myUT );
            myBarometricSensor.BMP085_TriggerPressure    ( BMP085::PRESSURE_STANDARD_MODE );
            myState = 2;
            break;
 
        case 2:
        // Read the uncompensated pressure data, calculate the compensated temperature and pressure and send it through the UART
            myled = 1;
            myBarometricSensor.BMP085_ReadRawPressure    ( &myUP );
 
            myTrueData = myBarometricSensor.BMP085_CalculateCompensated_Temperature_Pressure ( myCalCoeff, myUT, myUP, BMP085::PRESSURE_STANDARD_MODE );
 
            pc.printf( "Temperature: %0.1f\nPressure: %ld\r\n", ( float )myTrueData.Temperature/10, myTrueData.Pressure );
            myled = 0;
            myState = 0;
            break;
 
        default:
            myState = 0;
            break;
    }
 
}
 
 
int main()
{
    pc.baud ( 115200 );
 
    myBarometricSensor.BMP085_GetCalibrationCoefficients  ( &myCalCoeff );
 
    // Print the calibration coefficients
    pc.printf( "AC1: %ld\nAC2: %ld\nAC3: %ld\nAC4: %ld\nAC5: %ld\nAC6: %ld\nB1: %ld\nB2: %ld\nMB: %ld\nMC: %ld\nMD: %ld\r\n",
               myCalCoeff.AC1, myCalCoeff.AC2, myCalCoeff.AC3, myCalCoeff.AC4, myCalCoeff.AC5, myCalCoeff.AC6, myCalCoeff.B1,
               myCalCoeff.B2, myCalCoeff.MB, myCalCoeff.MC, myCalCoeff.MD );
 
 
    serial.attach( &sendDATA, 1 );                      // the address of the function to be attached ( sendDATA ) and the interval ( 1s )
 
    // Let the callbacks take care of everything
    while(1)  sleep();
}
*/
 
 
/*!
 Library for the BMP085 Digital Pressure Sensor.
*/
class BMP085
{
public:
    /**
      * @brief   DEFAULT ADDRESSES
      */
    typedef enum {
        BMP085_ADDRESS     =   ( 0x77 << 1 )
    } BMP085_address_t;
 
 
    /**
      * @brief   CALIBRATION COEFFICIENTS
      */
#define BMP085_AC1_MSB                  0xAA        /*!<   MSB AC1 coefficient                                          */
#define BMP085_AC1_LSB                  0xAB        /*!<   LSB AC1 coefficient                                          */
#define BMP085_AC2_MSB                  0xAC        /*!<   MSB AC2 coefficient                                          */
#define BMP085_AC2_LSB                  0xAD        /*!<   LSB AC2 coefficient                                          */
#define BMP085_AC3_MSB                  0xAE        /*!<   MSB AC3 coefficient                                          */
#define BMP085_AC3_LSB                  0xAF        /*!<   LSB AC3 coefficient                                          */
#define BMP085_AC4_MSB                  0xB0        /*!<   MSB AC4 coefficient                                          */
#define BMP085_AC4_LSB                  0xB1        /*!<   LSB AC4 coefficient                                          */
#define BMP085_AC5_MSB                  0xB2        /*!<   MSB AC5 coefficient                                          */
#define BMP085_AC5_LSB                  0xB3        /*!<   LSB AC5 coefficient                                          */
#define BMP085_AC6_MSB                  0xB4        /*!<   MSB AC6 coefficient                                          */
#define BMP085_AC6_LSB                  0xB5        /*!<   LSB AC6 coefficient                                          */
#define BMP085_B1_MSB                   0xB6        /*!<   MSB B1 coefficient                                           */
#define BMP085_B1_LSB                   0xB7        /*!<   LSB B1 coefficient                                           */
#define BMP085_B2_MSB                   0xB8        /*!<   MSB B2 coefficient                                           */
#define BMP085_B2_LSB                   0xB9        /*!<   LSB B2 coefficient                                           */
#define BMP085_MB_MSB                   0xBA        /*!<   MSB MB coefficient                                           */
#define BMP085_MB_LSB                   0xBB        /*!<   LSB MB coefficient                                           */
#define BMP085_MC_MSB                   0xBC        /*!<   MSB MC coefficient                                           */
#define BMP085_MC_LSB                   0xBD        /*!<   LSB MC coefficient                                           */
#define BMP085_MD_MSB                   0xBE        /*!<   MSB MD coefficient                                           */
#define BMP085_MD_LSB                   0xBF        /*!<   LSB MD coefficient                                           */
 
 
    /**
      * @brief   REGISTERS MAP
      */
#define BMP085_CONTROL                  0xF4        /*!<   Control register                                             */
 
 
 
    /* Commands Registers */
    /**
      * @brief   TEMPERATURE
      */
#define BMP085_TRIGGER_TEMPERATURE      0x2E        /*!<   Trigger a new Temperature measurement                        */
#define BMP085_READ_TEMPERATURE         0xF6        /*!<   Read Temperature                                             */
 
    /* Commands Registers */
    /**
      * @brief   PRESSURE
      */
#define BMP085_TRIGGER_PRESSURE         0x34        /*!<   Trigger a new Pressure measurement                           */
#define BMP085_READ_PRESSURE            0xF6        /*!<   Read Pressure                                                */
 
    typedef enum {
        PRESSURE_ULTRA_LOW_POWER_MODE     =   0,        /*!<  Pressure: Ultra low power mode.                                */
        PRESSURE_STANDARD_MODE            =   1,        /*!<  Pressure: Standard mode.                                       */
        PRESSURE_HIGH_RESOLUTION_MODE     =   2,        /*!<  Pressure: High resolution mode.                                */
        PRESSURE_ULTRA_HIGH_RES_MODE      =   3         /*!<  Pressure: Ultra high resolution mode.                          */
    } BMP085_pressure_osrs_t;
 
 
 
 
#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
    typedef struct {
        int16_t  AC1;
        int16_t  AC2;
        int16_t  AC3;
        uint16_t AC4;
        uint16_t AC5;
        uint16_t AC6;
        int16_t  B1;
        int16_t  B2;
        int16_t  MB;
        int16_t  MC;
        int16_t  MD;
    } Vector_cal_coeff_t;
 
 
    typedef struct {
        int16_t UT_Temperature;
    } Vector_temp_f;
 
    typedef struct {
        int32_t UP_Pressure;
    } Vector_pressure_f;
 
    typedef struct {
        int16_t Temperature;
        int32_t Pressure;
    } Vector_compensated_data_f;
#endif
 
 
    /**
      * @brief   INTERNAL CONSTANTS
      */
    typedef enum {
        BMP085_SUCCESS     =       0,
        BMP085_FAILURE     =       1,
        I2C_SUCCESS        =       0                                           /*!<   I2C communication was fine     */
    } BMP085_status_t;
 
 
 
 
    /** Create an BMP085 object connected to the specified I2C pins.
      *
      * @param sda     I2C data pin
      * @param scl     I2C clock pin
      * @param addr    I2C slave address
      * @param freq    I2C frequency in Hz.
      */
    BMP085 ( PinName sda, PinName scl, uint32_t addr, uint32_t freq );
 
    /** Delete BMP085 object.
     */
    ~BMP085();
 
    /** It gets the calibration coefficients.
     */
    BMP085_status_t  BMP085_GetCalibrationCoefficients    ( Vector_cal_coeff_t* myCalCoeff );
 
    /** It triggers a new temperature mesurement.
     */
    BMP085_status_t  BMP085_TriggerTemperature            ( void );
 
    /** It reads the raw temperature value.
     */
    BMP085_status_t  BMP085_ReadRawTemperature            ( Vector_temp_f* myRawTemperature );
 
    /** It reads the compensated/true temperature. NOT RECOMMENDED, use BMP085_CalculateCompensated_Temperature_Pressure instead!
     */
    BMP085_status_t  BMP085_ReadCompensatedTemperature    ( Vector_temp_f* myTrueTemperature, Vector_cal_coeff_t myCalCoeff );
 
    /** It triggers a new pressure mesurement.
     */
    BMP085_status_t  BMP085_TriggerPressure               ( BMP085_pressure_osrs_t myResolution );
 
    /** It reads the raw pressure value.
     */
    BMP085_status_t  BMP085_ReadRawPressure               ( Vector_pressure_f* myRawPressure );
 
    /** It calculates the compensated/true temperature and pressure values.
     */
    Vector_compensated_data_f  BMP085_CalculateCompensated_Temperature_Pressure ( Vector_cal_coeff_t myCalCoeff, Vector_temp_f myRawTemperature, Vector_pressure_f myRawPressure,
            BMP085_pressure_osrs_t myResolution );
 
private:
    I2C      i2c;
    uint32_t BMP085_Addr;
};
 
#endif
 