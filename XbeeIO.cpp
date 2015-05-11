/* ****************************************************************************
 * File Name:        XBeeDigital.c
 * Author:           Stephen Wayne
 * Created:          December 29, 2014
 * Revised:          September 4, 2014

 * Project Description: This is the driver file for an API Coordinator
 * XBee Series 2 module.
 *************************************************************************** */
 
 //#include <plib.h>
 #include <WProgram.h>
 
 #include "XbeeIO.h"
 
 void init_XBee(){ // initialize XBee
   XBee_Serial.begin(XBEE_DATA_RATE);
 }
 
 byte digi_check(word digi_mask){ // Check if digital pins enabled
  byte isDigital = 0; // reinitialize counter
  if(digi_mask & DIO0){
    Serial.println("DIO0");
    isDigital++;
  }
  if(digi_mask & DIO1){
    Serial.println("DIO1");
    isDigital++;
  }
   if(digi_mask & DIO2){
    Serial.println("DIO2");
    isDigital++;
  }
  if(digi_mask & DIO3){
    Serial.println("DIO3");
    isDigital++;
  }
  if(digi_mask & DIO4){
    Serial.println("DIO4");
    isDigital++;
  }
  if(digi_mask & DIO5){
    Serial.println("DIO5");
    isDigital++;
  }
  if(digi_mask & DIO6){
    Serial.println("DIO6");
    isDigital++;
  }
  if(digi_mask & DIO7){
    Serial.println("DIO7");
    isDigital++;
  }
  if(digi_mask & DIO10){
    Serial.println("DIO10");
    isDigital++;
  }
  if(digi_mask & DIO11){
    Serial.println("DIO11");
    isDigital++;
  }
  if(digi_mask & DIO12){
    Serial.println("DIO12");
    isDigital++;
  }
  if(!digi_mask){
    Serial.println("No Digital IO");
    isDigital = 0;
  }
  return isDigital;
}

// For Debugging purposes only. Serial output for which
// ADC pins are enabled on each XBee
void ana_check(byte ana_mask){
  
  if(ana_mask & ADC0)
    Serial.println("ADC0");
  if(ana_mask & ADC1)
    Serial.println("ADC1");
  if(ana_mask & ADC2)
    Serial.println("ADC2");
  if(ana_mask & ADC3)
    Serial.println("ADC3");
  if(!ana_mask)
    Serial.println("No Analog IO");
}

// Logic to check which pins are enabled on remote XBee and pull
// relevant data
String read_Xbee(byte XBee_buf[30]){
  
static String xbee_print;
float time_s = 0;
byte analogMSB = 0, analogLSB = 0;
word analogReading = 0;
byte ADC0_MSB = 19; //default first analog data byte
byte ADC1_MSB = ADC0_MSB;
byte ADC2_MSB = ADC1_MSB;
byte ADC3_MSB = ADC2_MSB;
float temp = 0;
//float pressure = 48; // pressure dummy variable
word digi_mask = 0; // digital IO mask
byte ana_mask = 0; // analog (ADC) input mask
byte isDigital = 0;// is there digital input on the XBees?
  
  digi_mask = XBee_buf[17] | (XBee_buf[16] << 8); // digital mask
  ana_mask = XBee_buf[18]; // XBee analog channel mask
  isDigital = digi_check(digi_mask); //check which digital IO enabled
  ADC0_MSB = isDigital?21:19; //21 if digital pins, 19 otherwise
  ADC1_MSB = (ana_mask&ADC0)?(ADC0_MSB+2):ADC0_MSB;
  ADC2_MSB = (ana_mask&ADC1)?(ADC1_MSB+2):ADC1_MSB;
  ADC3_MSB = (ana_mask&ADC2)?(ADC2_MSB+2):ADC2_MSB;
  ana_check(ana_mask);

    if((XBee_buf[XBEE_ADDR_LSBy]==R1_LSBy)||
        (XBee_buf[XBEE_ADDR_LSBy]==R2_LSBy)||(XBee_buf[XBEE_ADDR_LSBy]==R3_LSBy)||(XBee_buf[XBEE_ADDR_LSBy]==R4_LSBy)){
      Serial.print(String(XBee_buf[XBEE_ADDR_LSBy],HEX) + ": ");
      time_s = (millis())/1000.0; // Time in seconds
      Serial.println(String(time_s, DEC) + " Seconds");
      if(ana_mask & ADC0){//if there is a pressure sensor
        analogMSB = XBee_buf[ADC0_MSB];
        analogLSB = XBee_buf[ADC0_MSB+1];
        analogReading = analogLSB | (analogMSB << 8);
        pressure = get_pressure(analogReading,'R');
      }
      if(ana_mask & ADC1){//if there is a temperature sensor
        analogMSB = XBee_buf[ADC1_MSB];
        analogLSB = XBee_buf[ADC1_MSB+1];
        analogReading = analogLSB | (analogMSB << 8);
        temp = getTemp_F_LM19(analogReading,'R'); // indicate remote data
      }
      if(ana_mask & ADC2){
        
      }
      if(ana_mask & ADC3){
        
      }
      xbee_print = String(String(millis(),DEC) + "," + 
          String(XBee_buf[XBEE_ADDR_LSBy], HEX) + "," + String(temp, DEC)
            + "," + String(pressure,DEC));
    }
    else{
      Serial.println("Unknown Xbee");
      xbee_print = "UnknownXbee";
    }
    return xbee_print;
}

float get_pressure(word analogPres, char charlie){
  float tempPres = 0;
  if(charlie == 'R'){ // if remote data
    tempPres = analogPres*1.220/1023.0; // voltage at resistor divider output
    tempPres = tempPres * (R1_PRES_VAL + R2_PRES_VAL) / R1_PRES_VAL; // voltage at output
  }
  else if(charlie == 'L'){ // if local data
    tempPres = analogPres*3.3000/1023.0; //voltage measured at pressure output pin
  }
  tempPres = 18.75*((tempPres/V_Sup)-0.1);
  Serial.println(String(tempPres,DEC) + " PSIa");
  return tempPres;
}
