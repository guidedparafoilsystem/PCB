/* ****************************************************************************
 * File Name:        MS5803_12BA.cpp
 * Author:           Stephen Wayne
 * Created:          February 11, 2014
 * Revised:          

 * Project Description: This is the driver file for the digital temp/pressure
 * sensor.
 *************************************************************************** */
#include <MS5803_I2C.h>
//#include <Wire.h>
//#include <WProgram.h>
#include "MS5803_12BA.h"

 
MS5803 sensor(ADDRESS_HIGH);

//Create variables to store results
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
double base_altitude = 1655.0; // Altitude of SparkFun's HQ in Boulder, CO. in (m)

void init_MS5803_12BA(void){
  //Retrieve calibration constants for conversion math.
  sensor.reset();
  sensor.begin();
  pressure_baseline = sensor.getPressure(ADC_4096);
}

String poll_MS5803_12BA(void)
{
  // Read temperature from the sensor in deg C. This operation takes about 
  temperature_c = sensor.getTemperature(CELSIUS, ADC_512);
  // Read temperature from the sensor, convert to deg F
  temperature_f = sensor.getTemperature(FAHRENHEIT, ADC_512);
  // Read pressure from the sensor in mbar.
  pressure_abs = sensor.getPressure(ADC_4096);
  // Convert abs pressure with the help of altitude into relative pressure
  // This is used in Weather stations.
  pressure_relative = sealevel(pressure_abs, base_altitude);
  // Taking our baseline pressure at the beginning we can find an approximate
  // change in altitude based on the differences in pressure.   
  altitude_delta = altitude(pressure_abs , pressure_baseline);
   
  Serial.print("Temperature F = ");
  Serial.println(temperature_f);

  Serial.print("Pressure abs (mbar)= ");
  Serial.println(pressure_abs);

  Serial.print("Pressure relative (mbar)= ");
  Serial.println(pressure_relative); 

  Serial.print("Altitude change (m) = ");
  Serial.println(altitude_delta);
    
  return String(String(temperature_f, DEC) + "," + String(pressure_abs, DEC) + "," + String(pressure_relative,DEC) + "," + String(altitude_delta,DEC));
}

double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
  return(44330.0*(1-pow(P/P0,1/5.255)));
}

double sealevel(double P, double A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  return(P/pow(1-(A/44330.0),5.255));
}

float getTemp_F(word analogTemp){
  float temp = 0;
  temp = analogTemp / 1023.0 * 1.2;
  temp = temp - 0.5;
  temp = temp / 0.01;
  temp = temp * 9/5 + 32;
  return temp;
}

float getTemp_F_LM19(word analogTemp, char charlie){
  float tempC = 0;
  float tempF = 0;
  if(charlie == 'R'){ // if remote data
    tempC = analogTemp*1.260/1023.0; //voltage measured at resistor divider. Compensate for extra volt drop
    tempC = tempC * (R1_VAL + R2_VAL) / R1_VAL;
  }
  else if(charlie == 'L'){ //if local data
    tempC = analogTemp*3.300/1023.0;
  }
  tempC = -1481.96+sqrt(2196200.0+(1.8639-tempC)/(0.00000388));
  tempF = tempC * 1.80 + 32.0;
  Serial.print("Degrees F: ");
  Serial.println(tempF);
  return tempF;
}
