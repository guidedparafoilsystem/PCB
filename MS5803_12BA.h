/* ****************************************************************************
 * File Name:        MS5803_12BA.h
 * Author:           Stephen Wayne
 * Date:             February 11, 2015
 * Revised:          

 * Header Description:
 *************************************************************************** */
 
#ifndef MS5803_12BA_H
  #define MS5803_12BA_H
   
  #include <Wire.h>
  #include "Arduino.h"
  #include <WProgram.h>
  #include <MS5803_I2C.h>
  
  #define R1_VAL 97900
  #define R2_VAL 198000
  
  void init_MS5803_12BA(void);
  float getTemp_F(word);
  float getTemp_F_LM19(word,char);
  double altitude(double, double);
  double sealevel(double, double);
  String poll_MS5803_12BA(void);

#endif
