	

/**
 * @brief       BMP085.c
 * @details     Digital pressure sensor.
 *              Functions file.
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
 
 #include "BMP085.h"
 
 
BMP085::BMP085 ( PinName sda, PinName scl, uint32_t addr, uint32_t freq )
        : i2c          ( sda, scl )
        , BMP085_Addr  ( addr )
{
    i2c.frequency( freq );
}
 
 
BMP085::~BMP085(){
}
 
 
 
/**
 * @brief       BMP085_GetCalibrationCoefficients   ( Vector_cal_coeff_t* )
 *
 * @details     It gets the calibration coefficients.
 *
 * @param[in]    NaN
 *
 * @param[out]   Vector_cal_coeff_t:    Calibration coefficients.
 *
 *
 * @return       Status of BMP085_GetCalibrationCoefficients.
 *
 *
 * @author      Manuel Caballero
 * @date        25/August/2017
 * @version     25/August/2017   The ORIGIN
 * @pre         NaN
 * @warning     NaN.
 */
BMP085::BMP085_status_t  BMP085::BMP085_GetCalibrationCoefficients  ( Vector_cal_coeff_t* myCalCoeff )
{
    char        cmd[22]             =   { 0 };
    uint32_t    aux                 =    0;
 
    cmd[0]   =   BMP085_AC1_MSB;
    
    
    aux = i2c.write ( BMP085_Addr, &cmd[0], 1 );
    aux = i2c.read  ( BMP085_Addr, &cmd[0], 22 );
 
    // Parse the data
    myCalCoeff->AC1  =   ( cmd[0]  << 8 ) | cmd[1];
    myCalCoeff->AC2  =   ( cmd[2]  << 8 ) | cmd[3];
    myCalCoeff->AC3  =   ( cmd[4]  << 8 ) | cmd[5];
    myCalCoeff->AC4  =   ( cmd[6]  << 8 ) | cmd[7];
    myCalCoeff->AC5  =   ( cmd[8]  << 8 ) | cmd[9];
    myCalCoeff->AC6  =   ( cmd[10] << 8 ) | cmd[11];
    myCalCoeff->B1   =   ( cmd[12] << 8 ) | cmd[13];
    myCalCoeff->B2   =   ( cmd[14] << 8 ) | cmd[15];
    myCalCoeff->MB   =   ( cmd[16] << 8 ) | cmd[17];
    myCalCoeff->MC   =   ( cmd[18] << 8 ) | cmd[19];
    myCalCoeff->MD   =   ( cmd[20] << 8 ) | cmd[21];
 
 
 
    if ( aux == I2C_SUCCESS )
       return   BMP085_SUCCESS;
    else
       return   BMP085_FAILURE;
}
 
 
/**
 * @brief       BMP085_TriggerTemperature   ( void )
 *
 * @details     It triggers a new temperature measurement.
 *
 * @param[in]    NaN.
 *
 * @param[out]   NaN.
 *
 *
 * @return       Status of BMP085_TriggerTemperature.
 *
 *
 * @author      Manuel Caballero
 * @date        25/August/2017
 * @version     25/August/2017   The ORIGIN
 * @pre         NaN
 * @warning     The user MUST respect the maximum temperature conversion time, in this
 *              case, that value is 4.5ms.
 */
BMP085::BMP085_status_t  BMP085::BMP085_TriggerTemperature  ( void )
{
    char        cmd[]             =   { BMP085_CONTROL, BMP085_TRIGGER_TEMPERATURE };
    uint32_t    aux               =    0;
 
 
    aux = i2c.write ( BMP085_Addr, &cmd[0], 2 );
 
 
 
    if ( aux == I2C_SUCCESS )
       return   BMP085_SUCCESS;
    else
       return   BMP085_FAILURE;
}
 
 
/**
 * @brief       BMP085_ReadRawTemperature   ( Vector_temp_f* )
 *
 * @details     It reads an uncompensated temperature.
 *
 * @param[in]    NaN.
 *
 * @param[out]   myRawTemperature:      Uncompensated temperature.
 *
 *
 * @return       Status of BMP085_ReadRawTemperature.
 *
 *
 * @author      Manuel Caballero
 * @date        25/August/2017
 * @version     25/August/2017   The ORIGIN
 * @pre         BMP085_TriggerTemperature function MUST be called before calling this one.
 * @warning     The user MUST respect the maximum temperature conversion time, in this
 *              case, that value is 4.5ms.
 */
BMP085::BMP085_status_t  BMP085::BMP085_ReadRawTemperature  ( Vector_temp_f* myRawTemperature )
{
    char        cmd[]             =   { BMP085_READ_TEMPERATURE, 0 };
    uint32_t    aux               =    0;
 
 
    aux = i2c.write ( BMP085_Addr, &cmd[0], 1 );
    aux = i2c.read  ( BMP085_Addr, &cmd[0], 2 );
 
 
    // Parse the data
    myRawTemperature->UT_Temperature    =   ( cmd[0] << 8 ) | cmd[1];
 
 
 
    if ( aux == I2C_SUCCESS )
       return   BMP085_SUCCESS;
    else
       return   BMP085_FAILURE;
}
 
 
/**
 * @brief       BMP085_ReadCompensatedTemperature   ( Vector_temp_f*, Vector_cal_coeff_t )
 *
 * @details     It reads an compensated/true temperature.
 *
 * @param[in]    myCalCoeff:            Calibration coefficients.
 *
 * @param[out]   myTrueTemperature:      Compensated/True temperature.
 *
 *
 * @return       Status of BMP085_ReadCompensatedTemperature.
 *
 *
 * @author      Manuel Caballero
 * @date        25/August/2017
 * @version     25/August/2017   The ORIGIN
 * @pre         Both BMP085_TriggerTemperature and BMP085_GetCalibrationCoefficients functions MUST be called before calling this one.
 * @warning     The user MUST respect the maximum temperature conversion time, in this
 *              case, that value is 4.5ms.
 */
BMP085::BMP085_status_t  BMP085::BMP085_ReadCompensatedTemperature ( Vector_temp_f* myTrueTemperature, Vector_cal_coeff_t myCalCoeff )
{
    uint32_t    aux               =    0;
    int32_t     X1, X2, B5;
 
    Vector_temp_f myRawTemperature;
 
 
    aux = BMP085_ReadRawTemperature ( &myRawTemperature );
 
 
    // Parse the data
    X1   =   ( ( myRawTemperature.UT_Temperature - myCalCoeff.AC6 ) * myCalCoeff.AC5 ) / 32768;
    X2   =   ( myCalCoeff.MC * 2048 ) / ( X1 + myCalCoeff.MD );
    B5   =   X1 + X2;
 
    myTrueTemperature->UT_Temperature   =   ( B5 + 8 ) / 16;
 
 
    if ( aux == I2C_SUCCESS )
       return   BMP085_SUCCESS;
    else
       return   BMP085_FAILURE;
}
 
 
 
/**
 * @brief       BMP085_TriggerPressure   ( BMP085_pressure_osrs_t )
 *
 * @details     It triggers a new pressure measurement.
 *
 * @param[in]    myResolution:          Pressure resolution.
 *
 * @param[out]   NaN.
 *
 *
 * @return       Status of BMP085_TriggerPressure.
 *
 *
 * @author      Manuel Caballero
 * @date        25/August/2017
 * @version     25/August/2017   The ORIGIN
 * @pre         NaN
 * @warning     The user MUST respect the maximum pressure conversion time:
 *                  Ultra Low Power Mode:   4.5ms.
 *                  Standard Mode:          7.5ms.
 *                  High Resolution Mode:   13.5ms.
 *                  Ultra High Res. Mode:   25.5ms.
 */
BMP085::BMP085_status_t  BMP085::BMP085_TriggerPressure  ( BMP085_pressure_osrs_t myResolution )
{
    char        cmd[]             =   { BMP085_CONTROL, BMP085_TRIGGER_PRESSURE };
    uint32_t    aux               =    0;
 
    // adjust the pressure resolution
    cmd[1]   |=   ( myResolution << 6 );
 
 
    aux = i2c.write ( BMP085_Addr, &cmd[0], 2 );
 
 
 
    if ( aux == I2C_SUCCESS )
       return   BMP085_SUCCESS;
    else
       return   BMP085_FAILURE;
}
 
 
/**
 * @brief       BMP085_ReadRawPressure   ( Vector_pressure_f* )
 *
 * @details     It reads an uncompensated temperature.
 *
 * @param[in]    NaN.
 *
 * @param[out]   myRawPressure:         Uncompensated temperature.
 *
 *
 * @return       Status of BMP085_ReadRawPressure.
 *
 *
 * @author      Manuel Caballero
 * @date        25/August/2017
 * @version     25/August/2017   The ORIGIN
 * @pre         BMP085_TriggerPressure function MUST be called before calling this one.
 * @warning     The user MUST respect the maximum pressure conversion time:
 *                  Ultra Low Power Mode:   4.5ms.
 *                  Standard Mode:          7.5ms.
 *                  High Resolution Mode:   13.5ms.
 *                  Ultra High Res. Mode:   25.5ms.
 */
BMP085::BMP085_status_t  BMP085::BMP085_ReadRawPressure ( Vector_pressure_f* myRawPressure )
{
    char        cmd[]             =   { BMP085_READ_PRESSURE, 0, 0 };
    uint32_t    aux               =    0;
 
 
    aux = i2c.write ( BMP085_Addr, &cmd[0], 1 );
    aux = i2c.read  ( BMP085_Addr, &cmd[0], 3 );
 
 
    // Parse the data
    myRawPressure->UP_Pressure    =   ( cmd[0] << 16 ) | ( cmd[1] << 8 ) | cmd[2];
 
 
 
    if ( aux == I2C_SUCCESS )
       return   BMP085_SUCCESS;
    else
       return   BMP085_FAILURE;
}
 
 
/**
 * @brief       BMP085_CalculateCompensated_Temperature_Pressure   ( Vector_cal_coeff_t , Vector_temp_f , Vector_pressure_f , BMP085_pressure_osrs_t )
 *
 * @details     It reads an compensated pressure.
 *
 * @param[in]    myCalCoeff:            Calibration coefficients.
 * @param[in]    myRawTemperature:      Uncompensated temperature data.
 * @param[in]    myRawPressure:         Uncompensated pressure data.
 * @param[in]    myResolution:          Pressure resolution.
 *
 * @param[out]   NaN.
 *
 *
 * @return       Compensated Temperature and Pressure.
 *
 *
 * @author      Manuel Caballero
 * @date        25/August/2017
 * @version     25/August/2017   The ORIGIN
 * @pre         BMP085_GetCalibrationCoefficients, BMP085_ReadRawPressure and BMP085_ReadRawTemperature MUST be called before using this function.
 * @warning     NaN
 */
BMP085::Vector_compensated_data_f  BMP085::BMP085_CalculateCompensated_Temperature_Pressure ( Vector_cal_coeff_t myCalCoeff, Vector_temp_f myRawTemperature, Vector_pressure_f myRawPressure,
                                                                                              BMP085_pressure_osrs_t myResolution )
{
    int32_t     B6, X1, B5, X2, X3, B3;
    uint32_t    B4, B7;
 
    Vector_compensated_data_f myTrueData;
 
    // Calculate true temperature
    X1   =   ( ( myRawTemperature.UT_Temperature - myCalCoeff.AC6 ) * myCalCoeff.AC5 ) / 32768;
    X2   =   ( myCalCoeff.MC * 2048 ) / ( X1 + myCalCoeff.MD );
    B5   =   X1 + X2;
 
    myTrueData.Temperature   =   ( B5 + 8 ) / 16;
 
 
    // Calculate true pressure
    B6   =   B5 - 4000;
    X1   =   ( myCalCoeff.B2 * ( B6 * B6 / 4096 ) ) /2048;
    X2   =   myCalCoeff.AC2 * B6 / 2048;
    X3   =   X1 + X2;
    B3   =   ( ( ( myCalCoeff.AC1 * 4 + X3 ) << myResolution ) + 2 ) / 4;
    X1   =   myCalCoeff.AC3 * B6 / 8192;
    X2   =   ( myCalCoeff.B1 * ( B6 * B6 / 4096 ) ) / 65536;
    X3   =   ( ( X1 + X2 ) + 2 ) / 4;
    B4   =   myCalCoeff.AC4 * ( uint32_t )( X3 + 32768 ) / 32768;
    B7   =   ( ( uint32_t )myRawPressure.UP_Pressure - B3 ) * ( 50000 >> myResolution );
 
    if ( B7 < 0x80000000 )
        myTrueData.Pressure    =   ( B7 * 2 ) / B4;
    else
        myTrueData.Pressure    =   ( B7 / B4 ) * 2;
 
    X1   =   ( myTrueData.Pressure / 256 ) * ( myTrueData.Pressure / 256 );
    X1   =   ( X1 * 3038 ) / 65536;
    X2   =   ( -7357 * myTrueData.Pressure ) / 65536;
 
    myTrueData.Pressure   =   myTrueData.Pressure + ( X1 + X2 + 3791 ) / 16;
 
 
    return   myTrueData;
}