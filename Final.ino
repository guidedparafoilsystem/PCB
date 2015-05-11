#include "XbeeIO.h"
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>
#include <MS5803_I2C.h>
#include "MS5803_12BA.h" // Driver libraries for various components
#include "IMU_9DOF.h"
#include "GPS.h"
#include "Iridium.h"
#include "simple_control.h"

boolean sdWrite(File,String,String);
boolean delayMs(long int, int);
void init_SD(void);

const int chipSelect = 15; //microSD chip select line
const int analogTemp = 22; // analog temperature input
const int analogPress = 23; // analog pressure input
File xbeeFile, logFile, gpsFile, iridFile; // files to be written to SD card
String xbeeFile_name = "xbeefile",logFile_name = "logfile", gpsFile_name = "gpsfile"; //8 characters or fewer
String iridFile_name = "iridfile"; // file names for SD card

int i;

void setup(){
  Wire.begin(); //initialize I2C communication
  Serial.begin(115200); // initialize debugging (USB) serial
  init_Iridium(); // initialize Iridium 9603 modem
  init_XBee(); // initialize Coordinator API xbee
  init_MS5803_12BA(); //Initialize Digital Temp/Pressure Sensor
  init_IMU_9DOF(); //Initialize Accelerometer, Gyroscope, Magnetometer
  init_GPS(); //Initialize GPS
  init_SD(); //Initialize microSD card
  init_Servos(); // initialize servos, reset position
  delay(1000); //wait for everything to initialize
}

void loop(){
static String xbee_print, datalog_print, dl_print_tmp, imu, GPS_data;
String iridPacket = "", receivePacket = "";
static int numBytes;
static byte XBee_buf[30];
static long int last_poll_pres = 0;
static long int last_poll_imu = 0;
static long int last_poll_gps = 0;
static long int last_poll_iridium = 0;
static int poll_time_pres = 2000; //milliseconds between polling Pressure and Temp
static int poll_time_imu = 1000; //milliseconds between polling IMU
static int poll_time_gps = 2000; //milliseconds between polling GPS
static int poll_time_iridium = 15000;
bool newGPS_data = false;
word pressVal = 0;
word tempVal = 0;
float tempF = 0;
float tempPress = 0;

float lat;
float longt;

  newGPS_data = poll_GPS(&GPS_data); // check if new data and update string
  if(newGPS_data && delayMs(last_poll_gps,poll_time_gps)){
    last_poll_gps = millis();
    Serial.println("New GPS Data:"); // debugging (next line also)
    Serial.println(GPS_data);
    sdWrite(gpsFile,gpsFile_name,GPS_data); // write updated GPS data
    flight_control(GPS_data); // update winches for navigation
  }

  if(delayMs(last_poll_pres, poll_time_pres)){ // Poll local sensors
    last_poll_pres = millis();
    datalog_print = poll_MS5803_12BA();
    datalog_print = String(String(last_poll_pres,DEC) + "," + datalog_print);
    dl_print_tmp = datalog_print;
    imu = read_IMU_9DOF();
    datalog_print = datalog_print + "," + imu;
    pressVal = analogRead(analogPress);
    tempPress = get_pressure(pressVal,'L'); // local
    tempVal = analogRead(analogTemp);
    tempF = getTemp_F_LM19(tempVal,'L'); // local
    datalog_print = datalog_print + "," + String(tempF,DEC) + "," + String(tempPress,DEC);
    sdWrite(logFile, logFile_name, datalog_print);
  }

  if(XBee_Serial.available()>20){ // if complete XBee receive data frame
    numBytes = XBee_Serial.available(); // length of data frame
    for(int i = 0; i < numBytes; i++){
      XBee_buf[i] = XBee_Serial.read(); // loop and store xbee Rx data in buffer
      if(XBee_buf[i] == -1){ // check if over-run
        Serial.println("No more bytes to read");
        Serial.println(numBytes);
        for(int j = 0; j < numBytes; j++){
          Serial.print(XBee_buf[j]);
        }
      }
    }
    Serial.println();
    xbee_print = read_Xbee(XBee_buf); // pull relevant data
    sdWrite(xbeeFile, xbeeFile_name, xbee_print);
    Serial.println();
  }
  
  if(delayMs(last_poll_iridium, poll_time_iridium)){
    last_poll_iridium = millis();
    iridPacket = String(GPS_data + "," + dl_print_tmp); // join strings, CSV
    receivePacket = processIridium(iridPacket); // send/receive data
    if(strcmp(receivePacket.c_str(),"None")){ // if there was a received string
      Serial.println("String received from Iridium"); // debug
      receivePacket = String(last_poll_iridium,DEC) + "," + "Rx" + "," + receivePacket;
      sdWrite(iridFile,iridFile_name,receivePacket); // store received data
      // could add functionality to send commands to capsule here
    } // only do this if it actually sends through iridium
    iridPacket = String(last_poll_iridium,DEC) + "," + "Tx" + "," + iridPacket;
    sdWrite(iridFile,iridFile_name,iridPacket); // store transmitted data
  }
}

void init_SD(void){ // initialize SD card
  pinMode(chipSelect, OUTPUT); // SD card on SPI
  if(!SD.begin(chipSelect)){ // if initialization error
    Serial.println("initialization failed!");
    return;
  }
  else{
    Serial.println("SD initialization successful");
    sdWrite(xbeeFile, xbeeFile_name, String("time(ms),xbee_addr,temp(f),press(psi)"));
    sdWrite(logFile, logFile_name, String("time_ms,dTemp(f),pres_abs(mbar),press_rel(mbar),alt_change(m),accelx,accely,accelz,magx,magy,magz,gyrox,gyroy,gyroz,aTemp(f),aPres(psi)"));
    sdWrite(gpsFile, gpsFile_name, String("identifier,time,lat,dir,long,dir,fix,numSat,precision,alt,meter,height,meter,blank,check,vel(knots)"));
  }
}

// Function to easily write data to SD card
boolean sdWrite(File writeFile, String fileName, String write_string){
  String temp = String(fileName) + ".txt";
  char filename[temp.length()+1];
  temp.toCharArray(filename, sizeof(filename));
  writeFile = SD.open(filename, FILE_WRITE);
  writeFile.println(write_string);
  writeFile.close();
}

boolean delayMs(long int last_poll, int poll_time){ // Used for non-blocking delays
  if(millis() >= (last_poll + poll_time)) return true;
  else return false;
}
