/* ****************************************************************************
 * File Name:        Iridium.cpp
 * Original Author:  Gabe Pearhill
 * Modifying Author: Stephen Wayne
 * Date Created:     April 3, 2015
 * Revised:          

 * Project Description: This is the driver file for the Iridium 9603 SBD modem
 *************************************************************************** */
#include "Iridium.h"
#include "WCharacter.h"

const int iridiumPowerOn = 2;
const unsigned long IR_DELAY_TIME = 30000;

elapsedMillis iridiumTimer;

enum IridiumState {
        IR_DELAY,
        IR_ASKING_TO_SEND,
        IR_SENDING_DATA,
        IR_TRANSMITTING
};

enum IridiumResponse {
        IR_READY,
        IR_OK,
        IR_TRANS_RESP,
        IR_ERROR,
        IR_NULL,
        CUTDOWN,
        IR_ZERO
};

bool actionPerformed = false;
        
IridiumState iridiumState;

String iridiumResponse = "";

bool cutdown = false;

void init_Iridium(void){
  IRIDIUM_Serial.begin(IRIDIUM_DATA_RATE); //Open Iridium Serial at specified data rate
  pinMode(iridiumPowerOn, OUTPUT);
  digitalWrite(iridiumPowerOn, HIGH); // turn on iridium
  iridiumState = IR_DELAY; //initialize iridium state machine
}

String processIridium(String packet){
String recPacket = "";
  // Iridium State Machine
  switch(iridiumState){
    case IR_DELAY: //Allow iridium to boot for data-sheet specified time
        Serial.println("Iridium Booting/Delay");
        if(iridiumTimer > IR_DELAY_TIME){
          iridiumTimer = 0;
          iridiumState = IR_ASKING_TO_SEND;
          actionPerformed = false;
        }
        break;
    case IR_ASKING_TO_SEND: // tell iridium it will receive a packet to send
        Serial.println("ASK To Send State");
        if(!actionPerformed){
          IRIDIUM_Serial.println("AT+SBDWT"); // only send 120 bytes
          actionPerformed = true;
        }
        if(iridiumTimer > 1000 | getIridiumResponse() != IR_NULL){
          iridiumTimer = 0;
          iridiumState = IR_SENDING_DATA;
          actionPerformed = false;
        }
        break;
    case IR_SENDING_DATA: // Sending Iridium the packet
        Serial.println("Sending Data State");
        if(!actionPerformed){
          IRIDIUM_Serial.print(packet); // only print 120 bytes max
          actionPerformed = true;
        }
        if ((iridiumTimer > 1000) | (getIridiumResponse() != IR_NULL)){
          iridiumTimer = 0;
          iridiumState = IR_TRANSMITTING;
          actionPerformed = false;
        }
        break;
    case IR_TRANSMITTING: //Transmit the packet
        Serial.println("Transmitting State");
        if(!actionPerformed){
          IRIDIUM_Serial.println("AT+SBDI");
          actionPerformed = true;
        }
        if((iridiumTimer > 15000) | (getIridiumResponse() != IR_NULL)){
          iridiumTimer = 0;
          iridiumState = IR_DELAY;
          actionPerformed = false;
        }
        break;
  }
  if(iridiumState != IR_SENDING_DATA){
    recPacket = checkForIncomingPackets();
  }
  return recPacket;
}

String checkForIncomingPackets(){
String receivePacket = "";
  IRIDIUM_Serial.println("AT+SBDRB"); // ask for mobile terminated buffer data
  delay(10);
  char mt[64]; //read mobile terminated buffer
  int index = 0;
  for(int i = 0; i < 64;i++){
    mt[i] = '\0';
  }
  for (int i = 0; i < 64; i++){
    if(IRIDIUM_Serial.available()){
      mt[index] = IRIDIUM_Serial.read(); //get packet
      index++;
    }
    else{
      delay(1);
    }
  }
  
  if(mt[2] == 'c' && mt[3] == 'u' && mt[4] == 't' && mt[5] == 'd' && mt[6] == 'o'
        && mt[7] == 'w' && mt[8] == 'n'){
          cutdown = true;
  } //check for cutdown. first two bytes are checksum info. trigger cutdown
  if(mt[2] != '\0') receivePacket = String(mt);
  else receivePacket = "None";
  
  return receivePacket;
}

int getIridiumResponse(){
  if(iridiumState != IR_DELAY){ //if Iridium has booted
    for(int i = 0; i < 64; i++){
      if(IRIDIUM_Serial.available()){
        iridiumResponse += char(IRIDIUM_Serial.read()); //get incoming byte
      }
    }
    if(iridiumResponse.indexOf("READY") >= 0){
      Serial.print("Got READY\n");
      return IR_READY;
    }
    else if(iridiumResponse.indexOf("OK") >= 0){
      Serial.print("Got OK\n");
      return IR_OK;
    }
    else if(iridiumResponse.indexOf("SBD") >= 0){
      Serial.print("Got SBD\n");
      return IR_TRANS_RESP;
    }
    else if (iridiumResponse.indexOf("0") >= 0){
      Serial.print("Got 0\n");
      return IR_ZERO;
    }
    else if(iridiumResponse.indexOf("cutdown") >= 0){
      Serial.print("Got CUTDOWN\n");
      return CUTDOWN;
    }
  }
return IR_NULL;
}
