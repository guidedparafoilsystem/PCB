/* ****************************************************************************
 * File Name:        XbeeDigital.h
 * Author:           Stephen Wayne
 * Date:             December 29, 2014
 * Revised:          April 27, 2015

 * Header Description:
 *************************************************************************** */
 
 #ifndef XBEEIO_H
  #define XBEEIO_H
   
  #include "Arduino.h"
  #include "MS5803_12BA.h"
  
  #define XBee_Serial Serial2
  #define XBEE_DATA_RATE 9600
   
  // Digital IO
  #define DIO0 0x01
  #define DIO1 0x02
  #define DIO2 0x04
  #define DIO3 0x08
  #define DIO4 0x10
  #define DIO5 0x20
  #define DIO6 0x40
  #define DIO7 0x80
  #define DIO10 0x400
  #define DIO11 0x800
  #define DIO12 0x1000
  
  // Analog IO
  #define ADC0 0x01
  #define ADC1 0x02
  #define ADC2 0x04
  #define ADC3 0x08

  #define R1_LSBy 0x04 //endpoint 1
  #define R2_LSBy 0x16 // endpoint 2
  #define R3_LSBy 0xA4 //endpoint 3
  #define R4_LSBy 0x0F // Coordinator
  #define XBEE_ADDR_LSBy 11 //least significant byte
  
  #define R1_PRES_VAL 50500 // voltage divider values, in ohms
  #define R2_PRES_VAL 98220
  
  #define V_Sup 3.3
  
  // Function Prototypes
  void init_XBee(void);
  byte digi_check(word);
  void ana_check(byte);
  String read_Xbee(byte XBee_buf[30]);
  float get_pressure(word,char);
  // End of XbeeIO.h

#endif
