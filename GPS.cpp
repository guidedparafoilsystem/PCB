/* ****************************************************************************
 * File Name:        GPS.cpp
 * Author:           Stephen Wayne
 * Created:          February 11, 2015
 * Revised:          

 * Project Description: This is the driver file for the xxx GPS module
 *************************************************************************** */
#include "GPS.h"
#include "WCharacter.h"

String GPS_Format = "$GPGGA"; // main GPS data
String Vel_Format = "$GPRMC"; // pull speed over ground data from this

void init_GPS(void){
  GPS_Serial.begin(GPS_DATA_RATE); //Open GPS Serial at specified data rate
}

bool poll_GPS(String *last_GPS) // main GPS function
{
static String curr_GPS, VelStr;
static bool newGPS_data = false, newVel_data = false;
const char s[2] = ","; // used to parse string
char *token;

  char c = GPS_Serial.read(); //read in GPS output char by char
  if(isAlphaNumeric(c)||(c=='\n')||(c==',')||(c=='$')||(c=='.')){ // if character is valid
    curr_GPS += c; //append to GPS buffer
  }
  if(c=='\n'){ //if the end of a line
    if(!strncmp(curr_GPS.c_str(),Vel_Format.c_str(),6)){ //pull velocity data
      newVel_data = true;
      token = strtok(const_cast<char*>(curr_GPS.c_str()),s);
      for(int i = 0; i < 7; i++){ // grab 8th CSV, which is speed over land
        token = ""; // clear the value initially
        token = strtok(NULL,s);
      }
      VelStr = token;
    }
    if(!strncmp(curr_GPS.c_str(),GPS_Format.c_str(),6)){ //if the correct sentence type
      newGPS_data = true; // indicate new data to store
      curr_GPS = strtok(const_cast<char*>(curr_GPS.c_str()),"\n"); // Trim newline from string
      if(newVel_data){
        *last_GPS = curr_GPS + s + VelStr + "\n"; //send GPGGA string back to main with velocity
        newVel_data = false;
      }
      else{
        *last_GPS = curr_GPS + ",NoNewVel" + "\n"; //send GPGGA string back to main w/o velocity
      }
      curr_GPS = ""; // reset GPS buffer
    }
    else{//Throw out anything else
      newGPS_data = false; //no new data to store
      curr_GPS = ""; //reset GPS buffer
    }
  }
  return newGPS_data; //return new data status
}
