/* ****************************************************************************
 * File Name:        GPS.h
 * Author:           Stephen Wayne
 * Date Created:     April 1, 2015
 * Revised:          
 *************************************************************************** */
 
 #ifndef GPS_H
  #define GPS_H
   
  #include "Arduino.h"
  
  #define GPS_DATA_RATE 9600
  #define GPS_Serial Serial3
  
  // Function Prototypes
  void init_GPS(void); //initialize GPS
  bool poll_GPS(String *string); //read data from GPS
  // End of GPS.cpp

#endif
