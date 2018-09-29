/********************************************************************
 * ORIGINAL : 
 *
 * SparkFunBME280.cpp
 * BME280 Arduino and Teensy Driver
 * Marshall Taylor @ SparkFun Electronics
 * May 20, 2015
 * https://github.com/sparkfun/BME280_Breakout
 * 
 * Resources:
 * Uses Wire.h for i2c operation
 * Uses SPI.h for SPI operation
 * 
 * Development environment specifics:
 * Arduino IDE 1.6.4
 * Teensy loader 1.23
 * 
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 * Please review the LICENSE.md file included with this example. If you have any questions 
 * or concerns with licensing, please contact techsupport@sparkfun.com.
 * Distributed as-is; no warranty is given.
 * 
 * ******************************************************************
 * December 2017 Paul van Haastrecht
 * 
 * Modified and adjusted for use on raspberry Pi 
 *
 * Development environment specifics:
 * Raspberry Pi / linux Jessie release
 * 
 * Resources / dependencies:
 * BCM2835 library (http://www.airspayce.com/mikem/bcm2835/)
 * 
 * *****************************************************************
 * September 2018 Paul van Haastrecht
 * adjusted to be used with the twowire library on the Raspberry Pi
 * 
 * Resources / dependencies:
 * BCM2835 library (http://www.airspayce.com/mikem/bcm2835/)
 * twowire library (https://github.com/paulvha/twowire)
 * 
 * *****************************************************************
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ******************************************************************/

# include "BME280.h"

/* global constructor for I2C (hardware of software) */ 
TwoWire TWI;

/****************************************************************************
 *
 *  Settings and configuration
 *
 ****************************************************************************/

/*! Constructor -- Specifies default configuration */

BME280::BME280( uint8_t I2CAddress )
{
    /* initialize settings  (can be overruled by calling program) */
    settings.commInterface = hard_I2C; // use BCM2835 
    settings.I2CAddress = I2CAddress;
    settings.sda = DEF_SDA;           // set default SDA 
    settings.scl = DEF_SCL;           // set default SCL
    settings.baudrate = 100;          // set default baudrate   
    settings.runMode = 3;             // Normal mode = 3, forced = 1 or 2
    settings.tStandby = 4;            // 500 ms
    settings.filter = 4;              // 16 filter coefficient (highest)
    settings.tempOverSample = 5;      // 5 = x 16 
    settings.pressOverSample = 5;     // 5 = x 16
    settings.humidOverSample = 5;     // 5 = x 16

    t_fine = 0;
}

/*********************************************************************
 * Initialized the Raspberry Pi
 * 
 * interface : 
 *  hard_I2C / 1 = hardware I2C
 *  soft_I2C / 0 = software I2C (default in CCS811)
 *********************************************************************/

uint8_t BME280::hw_init(bool interface, uint8_t verbose) {
    
    /* enable driver messages in case of level 3 verbose */
    if (verbose == 3) TWI.setDebug(true);
    
    /* initialize the I2C software
     * 
     * for different SDA : change 2 to wanted GPIO #
     * for different SCL : change 3 to wanted GPIO #
     */
    if (TWI.begin(interface, settings.sda, settings.scl) != TW_SUCCESS) {
        p_printf(RED, (char *) "Can't setup i2c pin!\n");
        return(ERROR);
    }

    /* set baudrate */
    TWI.setClock(settings.baudrate);

    return(SUCCESS);
}

/********************************************************************
 * close Hardware correctly on the Raspberry Pi
 ********************************************************************/
void BME280::hw_close() {
    TWI.close();
}

/********************************************************************
 *
 *  Configuration section
 *
 *  This uses the stored SensorSettings to start the IMU
 * 
 *  Use statements such as "mySensor.settings.runMode = 3;" to 
 *  configure before calling .begin();
 *  @ param ret : return the ID (0x60) 
 * 
 *  returns ERROR or SUCCESS.
 *
 ********************************************************************/
uint8_t BME280::begin( uint8_t *ret)
{
    uint8_t dataToWrite = 0;  // Temporary variable
    uint8_t temp1, temp2;     // temp variables

    /* Reading all compensation data, range 0x88:A1, 0xE1:E7 */

    if (readRegisterInt16(BME280_DIG_T1_LSB_REG, (int16_t *) &calibration.dig_T1) == ERROR) return(ERROR);
    if (readRegisterInt16(BME280_DIG_T2_LSB_REG, &calibration.dig_T2) == ERROR) return(ERROR);
    if (readRegisterInt16(BME280_DIG_T3_LSB_REG, &calibration.dig_T3) == ERROR) return(ERROR);
    
//printf("\nT1 %x, T2 %x, T3  %x\n",calibration.dig_T1, calibration.dig_T2,calibration.dig_T3);
     
    if (readRegisterInt16(BME280_DIG_P1_LSB_REG, (int16_t *) &calibration.dig_P1) == ERROR) return(ERROR);
    if (readRegisterInt16(BME280_DIG_P2_LSB_REG,  &calibration.dig_P2) == ERROR) return(ERROR);
    if (readRegisterInt16(BME280_DIG_P3_LSB_REG,  &calibration.dig_P3) == ERROR) return(ERROR); 
    if (readRegisterInt16(BME280_DIG_P4_LSB_REG,  &calibration.dig_P4) == ERROR) return(ERROR); 
    if (readRegisterInt16(BME280_DIG_P5_LSB_REG,  &calibration.dig_P5) == ERROR) return(ERROR);
    if (readRegisterInt16(BME280_DIG_P6_LSB_REG,  &calibration.dig_P6) == ERROR) return(ERROR);
    if (readRegisterInt16(BME280_DIG_P7_LSB_REG,  &calibration.dig_P7) == ERROR) return(ERROR);
    if (readRegisterInt16(BME280_DIG_P8_LSB_REG,  &calibration.dig_P8) == ERROR) return(ERROR);
    if (readRegisterInt16(BME280_DIG_P9_LSB_REG,  &calibration.dig_P9) == ERROR) return(ERROR);

//printf("\nP1 %x, P2 %x, P3 %x, P4 %x, P5 %x, P6 %x, P7 %x, P8 %x, P9 %x\n",calibration.dig_P1, calibration.dig_P2,calibration.dig_P3,
//calibration.dig_P4, calibration.dig_P5,calibration.dig_P6, calibration.dig_P7, calibration.dig_P8,calibration.dig_P9);
   
    if (readRegister(BME280_DIG_H1_REG, &calibration.dig_H1) != SUCCESS) return (ERROR);
    
    if (readRegister(BME280_DIG_H2_MSB_REG, &temp1) != SUCCESS) return (ERROR);
    if (readRegister(BME280_DIG_H2_LSB_REG, &temp2) != SUCCESS) return (ERROR);
    calibration.dig_H2 = (int16_t) ((temp1 << 8) + temp2);
    
    if (readRegister(BME280_DIG_H3_REG, &calibration.dig_H3) != SUCCESS) return (ERROR);
    
    if (readRegister(BME280_DIG_H4_MSB_REG, &temp1) != SUCCESS) return (ERROR);
    if (readRegister(BME280_DIG_H4_LSB_REG, &temp2) != SUCCESS) return (ERROR);
    calibration.dig_H4 = (int16_t)((temp1 << 4) + (temp2 & 0x0F));
     
    if (readRegister(BME280_DIG_H5_MSB_REG, &temp1) != SUCCESS) return (ERROR);
    if (readRegister(BME280_DIG_H4_LSB_REG, &temp2) != SUCCESS) return (ERROR);
    calibration.dig_H5 = (int16_t)((temp1 << 4) + ((temp2>>4) & 0xf));   
     
    if (readRegister(BME280_DIG_H6_REG, &calibration.dig_H6) != SUCCESS) return (ERROR);

//printf("\nH1 %x, H2 %x, H3 %x, H4 %x, H5 %x, H6 %x\n",calibration.dig_H1, calibration.dig_H2,calibration.dig_H3,
//calibration.dig_H4, calibration.dig_H5,calibration.dig_H6 );

    /*! set oversampling */
    
    /* config will only be writeable in sleep mode, so first insure that. */
    if(writeRegister(BME280_CTRL_MEAS_REG, 0x00) != SUCCESS)  return(ERROR);
    
    /* Set the config word */
    dataToWrite = (settings.tStandby << 0x5) & 0xE0;    // set inactive time in normal mode
    dataToWrite |= (settings.filter << 0x02) & 0x1C;    // set filter coefficient
    writeRegister(BME280_CONFIG_REG, dataToWrite);
    
    /* Set ctrl_hum first, then ctrl_meas to activate ctrl_hum */
    dataToWrite = settings.humidOverSample & 0x07; //all other bits can be ignored
    if (writeRegister(BME280_CTRL_HUMIDITY_REG, dataToWrite)!= SUCCESS)  return(ERROR);
    
    /*! set ctrl_meas */
    /* First, set temp oversampling */
    dataToWrite = (settings.tempOverSample << 0x5) & 0xE0;
    
    /* Next, pressure oversampling */
    dataToWrite |= (settings.pressOverSample << 0x02) & 0x1C;
    
    /* Last, set run mode (sleep = 0 , forced = 1 , normal = 3) */
    dataToWrite |= (settings.runMode) & 0x03;
    
    /* Load the byte */
    if (writeRegister(BME280_CTRL_MEAS_REG, dataToWrite) != SUCCESS)  return(ERROR);;

    /*! get Chip ID in ret variable*/
    if (readRegister(BME280_CHIP_ID_REG, ret) != SUCCESS)  return(ERROR);

    return (SUCCESS);
}

/*********************************************************************
 * Strictly software reset.  Run .begin() afterwards
 *******************************************************************/
void BME280::reset( void ) {
    writeRegister(BME280_RST_REG, 0xB6);
    bcm2835_delay(20);
}

/********************************************************************
 *
 *  sets runmode
 * 
 *  @param mode: run mode requested to set ( 0- 4)
 *
 ********************************************************************/
uint8_t BME280::set_run_mode( uint8_t mode)
{
    uint8_t control = 0;  //Temporary variable
    
    if(readRegister(BME280_CTRL_MEAS_REG, &control) != SUCCESS) return(ERROR);
    
    /* config will only be writeable in sleep mode, so first insure that. */
    control = control & 0xfc;   // set to sleep
    if(writeRegister(BME280_CTRL_MEAS_REG, control) != SUCCESS)  return(ERROR);

    /* set mode. */
    control |= mode & 0x03;
    if(writeRegister(BME280_CTRL_MEAS_REG, control) != SUCCESS)  return(ERROR);
        
    settings.runMode = mode;
    
    return(SUCCESS);
}

/*********************************************************************
 *
 *  Pressure Section
 *
 *  t_fine MUST have been set before by calling readTempC
 * 
 *  @param ret : return the value humidity
 * 
 *  returns SUCCESS or ERROR
 * 
 * There is NO explanation on how this calculation works. It is just 
 * following what the data-sheet indicated. 
 * 
 *********************************************************************/
uint8_t BME280::readFloatPressure( float *ret  )
{
    /* Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format 
     * (24 integer bits and 8 fractional bits).
     * Output value of “24674867” represents 
     *    24674867/256 = 96386.2 Pa = 963.862 hPa
     */
    uint8_t temp1, temp2, temp3;
    
    if (readRegister(BME280_PRESSURE_MSB_REG, &temp1) != SUCCESS) return(ERROR);
    if (readRegister(BME280_PRESSURE_LSB_REG, &temp2) != SUCCESS) return(ERROR);
    if (readRegister(BME280_PRESSURE_XLSB_REG,&temp3) != SUCCESS) return(ERROR);
    
    int32_t adc_P = (uint32_t) temp1 << 12 | (uint32_t) (temp2 << 4) | (uint32_t) ((temp3 >> 4) & 0x0F);
  
    int64_t var1, var2, p_acc;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calibration.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
    var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
    
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p_acc = 1048576 - adc_P;
    p_acc = (((p_acc<<31) - var2)*3125)/var1;
    var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
    var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
    p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);

    *ret = p_acc / 256.0;
    
    return (SUCCESS);
}

/********************************************************************
 * This calculates, based on the actual pressure, the height in meters
 * compared to sea level. It is assumed that the pressure at sealevel 
 * is 1013.25 hPa which is the standard defined. 
 * 
 * HOWEVER normally the air pressure is fluctuating and for this 
 * calculation to make sense, one should provide the current pressure 
 * at sealevel of a place nearby.
 *
 * The standard routine has been changed to. if seapressure = 0, the 
 * standard sealevel pressure of 1013.25 hPa is used, else the provided
 * seapressure is used. This pressure can be found on different web-sites
 * 
 * @param seapressure : current seapressure value to compare or 0 
 * to use standardized sealevel
 *  
 * @param ret : return the value in celcius
 * 
 * returns SUCCESS or ERROR
 * 
 ********************************************************************/
uint8_t BME280::readFloatAltitudeMeters( float seapressure, float *ret  )
{
    float heightOutput = 0, pressure;
    
    if (readFloatPressure(&pressure) == ERROR) return(ERROR);
    
    if (seapressure)
        heightOutput = ((float)44330)* (1-(pow(((float)pressure/ (float) seapressure), 0.190284)));
    else
        heightOutput = ((float)44330)* (1-(pow(((float)pressure /(float)101325), 0.190284)));

    *ret = heightOutput;

    return(SUCCESS);
}

/*********************************************************************
 *
 *  Read the height in feet
 * 
 * @param seapressure : current seapressure value to compare or 0 
 * to use standardized sealevel
 *  
 * @param ret : return the value in celcius
 * 
 * returns SUCCESS or ERROR
 *
 *********************************************************************/
uint8_t BME280::readFloatAltitudeFeet( float seapressure, float *ret )
{
    float heightOutput = 0;
    
    if (readFloatAltitudeMeters(seapressure, &heightOutput) == ERROR) return(ERROR);
    
    *ret = heightOutput * 3.28084;
    
    return(SUCCESS);
}

/**********************************************************************
 *
 *  Humidity Section
 * 
 *  t_fine MUST have been set before by calling readTempC
 * 
 *  @param ret : return the value humidity
 * 
 *  returns SUCCESS or ERROR
 * 
 * There is NO explanation on how this calculation works. It is just 
 * following what the data-sheet indicated. This code below is taken 
 * from the latest  BOSCH BME280.c 3.3.2 / 22 Nov 2017, and provides 
 * the same result as earlier versions.
 *********************************************************************/
uint8_t BME280::readFloatHumidity( float *ret )
{
    /* Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format 
     * (22 integer and 10 fractional bits). */
     
    uint8_t temp1, temp2;
    
    if (readRegister(BME280_HUMIDITY_MSB_REG, &temp1) != SUCCESS) return(ERROR);
    if (readRegister(BME280_HUMIDITY_LSB_REG, &temp2) != SUCCESS) return(ERROR);
    
    int32_t adc_H = (uint32_t) temp1 << 8 | (uint32_t )temp2;
    
    int32_t var1, var2, var3, var4, var5;
    int32_t humidity, humidity_max = 100000;
    
    var1 = t_fine - ((int32_t)76800);
    var2 = (int32_t)(adc_H * 16384);
    var3 = (int32_t)(((int32_t)calibration.dig_H4) * 1048576);
    var4 = ((int32_t)calibration.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calibration.dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)calibration.dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calibration.dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calibration.dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max)   humidity = humidity_max;

    *ret =(float) humidity /1024;

    return(SUCCESS);
}

/**********************************************************************
 *
 *  Temperature Section
 * 
 *  t_fine is set by this routine and MUST be set before calling 
 *  pressure or humidity
 * 
 *  @param ret : return the value in celcius
 * 
 *  returns SUCCESS or ERROR
 * 
 * There is NO explanation on how this calculation works. It is just 
 * following what the data-sheet indicated. 
 *********************************************************************/

uint8_t BME280::readTempC( float *ret )
{
    /* Returns temperature in DegC, resolution is 0.01 DegC.
     * t_fine carries fine temperature as global value */
    uint8_t temp1, temp2 , temp3;
    
    /* get the reading (adc_T); */
    if (readRegister(BME280_TEMPERATURE_MSB_REG, &temp1) != SUCCESS) return (ERROR);
    if (readRegister(BME280_TEMPERATURE_LSB_REG, &temp2) != SUCCESS) return (ERROR);
    if (readRegister(BME280_TEMPERATURE_XLSB_REG,&temp3) != SUCCESS) return (ERROR);

    int32_t adc_T = (uint32_t)(temp1 << 12) | (uint32_t)(temp2 << 4) |(uint32_t) ((temp3 >> 4) & 0x0F);

    /* By datasheet, calibrate */
    int64_t var1, var2;

    var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
    ((int32_t)calibration.dig_T3)) >> 14;
    t_fine = var1 + var2;
 
    float output = (t_fine * 5 + 128) >> 8;

    *ret = output / 100;
    
    return(SUCCESS);
}

/****************************************************************
 *  calculate temperature in Fahrenheit
 * 
 * @param ret : return the value in fahrenheit
 * 
 * returns SUCCESS or ERROR
 ****************************************************************/
uint8_t BME280::readTempF( float *ret )
{
    float output;
    if (readTempC(&output) != SUCCESS) return(ERROR);
    *ret = (output * 9) / 5 + 32;

    return (SUCCESS);
}

//////////////////////////////////////////////////////////////////
///// low level access routines //////////////////////////////////
//////////////////////////////////////////////////////////////////
/*********************************************************************
 *
 * read register Region . A region can be 1.
 * 
 * @param outputPointer : array to store the read registers
 * @param offset : start address to read from
 * @param length : number or registers to read
 * 
 * returns SUCCESS or ERROR
 *
 ********************************************************************/
uint8_t BME280::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
    int returnError = SUCCESS;
    
    /* set slave address for BME280 */
    TWI.setSlave(settings.I2CAddress);
 
    /* perform read with restart */
    switch(TWI.i2c_read_rs((char *) &offset, (char *) outputPointer, length))
    {
        case I2C_SDA_NACK :
            p_printf(RED, (char *) "NACK error\n");
            returnError = ERROR;
            break;
        case I2C_SCL_CLKSTR :
            p_printf(RED,(char *)"Clock stretch error\n");
            returnError = ERROR;
            break;
        case I2C_SDA_DATA :
            p_printf(RED,(char *)"not all data has been read\n");
            returnError = ERROR;
            break;
        case I2C_OK:
            break;
            
        default:
            p_printf(RED, (char *) "unkown return code\n");
            returnError = ERROR;
    }
    
    return(returnError);
}

/***************************************************************
 * read single register
 * 
 * @param offset : register address to read
 * @param result : store the read value
 * 
 * returns SUCCESS or ERROR
 * 
 **************************************************************/
uint8_t BME280::readRegister(uint8_t offset, uint8_t *result) {
   return(readRegisterRegion(result, offset, 1));
}

/************************************************************
 * read 2 registers (16 bits)
 * 
 * @param offset : start address to read
 * @param output : store 16 bit value
 * 
 * returns SUCCESS or ERROR
 ***********************************************************/
uint8_t BME280::readRegisterInt16( uint8_t offset, int16_t *output )
{
    uint8_t myBuffer[2];
    
    if (readRegisterRegion(myBuffer, offset, 2) != SUCCESS)
        return(ERROR);     
    
    *output = (int16_t) myBuffer[0] | int16_t (myBuffer[1] << 8);

    return(SUCCESS);
}

/**************************************************************
 *  write a register
 * 
 * @param offset : start address to write
 * @param dataToWrite : the 8 bits to write to the register
 * 
 * returns SUCCESS or ERROR
 ************************************************************/
uint8_t BME280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
    char    buff[2];
    int     returnError = SUCCESS;

    buff[0]=offset;
    buff[1]=dataToWrite;
    
    /* set slave address for BME280 */
    TWI.setSlave(settings.I2CAddress);
    
    switch(TWI.i2c_write(buff, 2))
    {
        case I2C_SDA_NACK :
            p_printf(RED,(char* )"write NACK error\n");
            returnError = ERROR;
            break;
        case I2C_SCL_CLKSTR :
            p_printf(RED,(char* )"write Clock stretch error\n");
            returnError = ERROR;
            break;
        case I2C_SDA_DATA :
            p_printf(RED,(char* )"write not all data has been sent\n");
            returnError = ERROR;
            break;
        case I2C_OK:
            break;
            
        default:
            p_printf(RED,(char* )"other error occured during write\n");
            returnError = ERROR;
            break;
    }
    
    return(returnError);
}
