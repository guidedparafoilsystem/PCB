/* ****************************************************************************
 * File Name:        IMU_9DOF.h
 * Author:           Stephen Wayne
 * Date:             February 11, 2015
 * Revised:          

 * Header Description:
 *************************************************************************** */
 
#ifndef IMU_9DOF_H
  #define IMU_9DOF_H
   
  #include <Wire.h>
  #include "Arduino.h"
  #include <WProgram.h>
  #include <MS5803_I2C.h>
  
  #define  ADXL345_ADDRESS (0xA6 >> 1)
  //There are 6 data registers, they are sequential starting 
  //with the LSB of X.  We'll read all 6 in a burst and won't
  //address them individually
  #define ADXL345_REGISTER_XLSB (0x32)

  //Need to set power control bit to wake up the adxl345
  #define ADXL_REGISTER_PWRCTL (0x2D)
  #define ADXL_PWRCTL_MEASURE (1 << 3)

  #define ITG3200_ADDRESS (0xD0 >> 1)
  //request burst of 6 bytes from this address
  #define ITG3200_REGISTER_XMSB (0x1D)
  #define ITG3200_REGISTER_DLPF_FS (0x16)
  #define ITG3200_FULLSCALE (0x03 << 3)
  #define ITG3200_42HZ (0x03)

  #define HMC5843_ADDRESS (0x3C >> 1)
  //First data address of 6 is XMSB.  Also need to set a configuration register for
  //continuous measurement
  #define HMC5843_REGISTER_XMSB (0x03)
  #define HMC5843_REGISTER_MEASMODE (0x02)
  #define HMC5843_MEASMODE_CONT (0x00)
  
  void init_IMU_9DOF(void);
  String read_IMU_9DOF(void);
  void i2c_write(int,byte,byte);
  void i2c_read(int,byte,int,byte*);
  void init_adxl345(void);
  void read_adxl345(void);
  void init_itg3200(void);
  void read_itg3200(void);
  void init_hmc5843(void);
  void read_hmc5843(void);

#endif
