/* ****************************************************************************
 * File Name:        Iridium.h
 * Original Author:  Gabe Pearhill
 * Modifying Author: Stephen Wayne
 * Date Created:     April 3, 2015
 * Revised:          

 * Header Description:
 *************************************************************************** */
 
 #ifndef IRIDIUM_H
  #define IRIDIUM_H
   
  #include "Arduino.h"
  
  #define IRIDIUM_DATA_RATE 19200
  #define IRIDIUM_Serial Serial1
  
  // Function Prototypes
  void init_Iridium(void); //initialize Iridium
  String processIridium(String packet); // Iridium State machine
  String checkForIncomingPackets(void);
  int getIridiumResponse(void);
  // End of Iridium.h

#endif
