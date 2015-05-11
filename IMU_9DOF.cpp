/* ****************************************************************************
 * File Name:        IMU_9DOF.cpp
 * Author:           Stephen Wayne
 * Created:          February 11, 2015
 * Revised:          

 * Project Description: This is the driver file for the 9DOF IMU from Sparkfun
 *************************************************************************** */
//#include <Wire.h>
//#include <WProgram.h>
#include "IMU_9DOF.h"


int accel_data[3];
int gyro_data[3];
int mag_data[3];


void init_IMU_9DOF(){
  for(int i = 0; i < 3; ++i) {
    accel_data[i] = mag_data[i] = gyro_data[i] = 5;
  }
  init_adxl345();
  init_hmc5843();
  init_itg3200();
  Serial.print("IMU Initialized");
}

String read_IMU_9DOF(){
  read_adxl345();
  Serial.print("ACCEL: ");
  Serial.print(accel_data[0]);
  Serial.print(",");
  Serial.print(accel_data[1]);
  Serial.print(",");
  Serial.println(accel_data[2]);
   
  read_hmc5843();
  Serial.print("MAG: ");
  Serial.print(mag_data[0]);
  Serial.print(",");
  Serial.print(mag_data[1]);
  Serial.print(",");
  Serial.println(mag_data[2]);

  read_itg3200();
  Serial.print("GYRO: ");
  Serial.print(gyro_data[0]);
  Serial.print(",");
  Serial.print(gyro_data[1]);
  Serial.print(",");
  Serial.println(gyro_data[2]);
  Serial.println();
  //Sample at 3.33Hz*/
  return String(String(accel_data[0],DEC)+","+String(accel_data[1],DEC)+","+String(accel_data[2],DEC)+","+String(mag_data[0],DEC)+","+String(mag_data[1],DEC)+","+String(mag_data[2],DEC)+","+String(gyro_data[0],DEC)+","+String(gyro_data[1],DEC)+","+String(gyro_data[2],DEC));
}

void i2c_write(int address, byte reg, byte data){
  // Send output register address
  Wire.beginTransmission(address);
  Wire.write(reg);
  // Connect to device and send byte
  Wire.write(data); // low byte
  Wire.endTransmission();
}

void i2c_read(int address, byte reg, int count, byte* data){
  int i = 0;

  // Send input register address
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  // Connect to device and request bytes
  
  Wire.beginTransmission(address);
  Wire.requestFrom(address,count);
  while(Wire.available()){ // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    data[i] = c;
    i++;
  }
  Wire.endTransmission();
}

void init_adxl345(){
  byte data = 0;

  i2c_write(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, ADXL_PWRCTL_MEASURE);

  //Check to see if it worked!
  i2c_read(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, 1, &data);
  Serial.println((unsigned int)data);
}

void read_adxl345(){
  byte bytes[6];
  memset(bytes,0,6);

  //read 6 bytes from the ADXL345
  i2c_read(ADXL345_ADDRESS, ADXL345_REGISTER_XLSB, 6, bytes);

  //now unpack the bytes
  for (int i=0;i<3;++i) {
    accel_data[i] = (int16_t)((int)bytes[2*i] + (((int)bytes[2*i + 1]) << 8));
  }
}

void init_itg3200(){
  byte data = 0;

  //Set DLPF to 42 Hz (change it if you want) and
  //set the scale to "Full Scale"
  i2c_write(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, ITG3200_FULLSCALE | ITG3200_42HZ);

  //Sanity check! Make sure the register value is correct.
  i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, 1, &data);

  Serial.println((unsigned int)data);
}

void read_itg3200(){
  byte bytes[6];
  memset(bytes,0,6);

  //read 6 bytes from the ITG3200
  i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_XMSB, 6, bytes);  //now unpack the bytes
  for (int i=0;i<3;++i) {
  gyro_data[i] = (int16_t)((int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8));
  }
}

void init_hmc5843(){
  byte data = 0;
  //set up continuous measurement
  i2c_write(HMC5843_ADDRESS, HMC5843_REGISTER_MEASMODE, HMC5843_MEASMODE_CONT);

  //Sanity check, make sure the register value is correct.
  i2c_read(HMC5843_ADDRESS, HMC5843_REGISTER_MEASMODE, 1, &data);
  Serial.println((unsigned int)data);
}

void read_hmc5843(){
  byte bytes[6];
  memset(bytes,0,6);

  //read 6 bytes from the HMC5843
  i2c_read(HMC5843_ADDRESS, HMC5843_REGISTER_XMSB, 6, bytes);

  //now unpack the bytes
  for (int i=0;i<3;++i) {
  mag_data[i] = (int16_t)((int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8));
  }
}
