/* 
 * File:   simple_control.h
 * Author: richardpark
 *
 * Created on April 25, 2015, 4:51 PM
 */

#ifndef SIMPLE_CONTROL_H
  #define	SIMPLE_CONTROL_H
  
  //***********************************************************************************************************************
  //Last updated 5/2/2012
  //Josh's targeted control algorithm
  //These are used by simple_control(), steerToTarget(), landingRoutine(), and landingFlare()
  
  #include "Arduino.h"
  
  typedef struct servo
  {
  	float value;
  	float value_failsafe;
  
        int converted_value;
        //int converted_failsafe;
        
  } servo;
  
  
  
  //Global variables:
  extern float headingAct[2]; //actual heading, initialize {0,0} here; updated in loop
  extern float servoOutPrcnt[2]; //intialize {0,0}; corrective steering command to servo without amplification scaling, a fn of headingDev
  extern float desiredYawRate[2]; //intialize {0,0}; minimum desired yaw rate, a fn of headingDev, degrees/s, must be L ESS THAN 180 deg/s or steering routine WILL FAIL
  extern int steeringActive; //intialize 0; 1 if steering occured previously this loop, 0 if not
  extern float steeringScale[2]; //intialize {0,0}; servo scaling factor to achieve desired yaw
  extern float achievedYawRate[2]; //intialize {0,0}; yaw rate achieved by previous turn command, degrees/s
  extern int steerDelayCounter; //intialize 0; counter for initial steering input lag delay
  extern int turnedLeft[2]; //intialize {0,0}; flag -- if true, system turned left last loop (but NOT as a result of negative servo value)
  extern int turnedRight[2]; //intialize {0,0}; flag -- if true, system turned right last loop (but NOT as a result of negative servo value)
  extern double headingTime[2]; //initialize {0,0}; holds GPS time-of-week (TOW) in milliseconds
  
  // (time, lat, long, satellites, alt, velocity)
  extern float gGPSheading;
  extern float gGPSlatitude;
  extern float gGPSlongitude;
  extern float gGPSgndspeed;
  extern float gGPSaltitude;
  extern double gGPSTOW;
  
  //Function prototypes:
  void init_Servos(void);
  void Servos_Update_All(void);
  void simple_control(void);
  void steerToTarget(float headingDes);
  void landingRoutine(float headingDes);
  void landingFlare(void);
  void flight_control(String);
  
  float convert_latitude_to_decimal_degrees(String latitude);
  float convert_longitude_to_decimal_degrees(String longitude);

  
  void test_servos(void);
  
  //Landing target parameters:
  //#define TARGETLAT 37.2862100 //target latitude in decimal degrees
  //#define TARGETLONG -121.8517000 //target longitude in decimal degrees
  //#define TARGET_ALTITUDE 72.1550 //elevation of landing target coordinates above sea level, meters
  
  #define TARGETLAT 46.852814//target latitude in decimal degrees
  #define TARGETLONG -117.619322 //target longitude in decimal degrees
  #define TARGET_ALTITUDE 504.0000 //elevation of landing target coordinates above sea level, meters
  
  //Distance and heading calculation constants:
  #define EARTHRAD 6371000 //radius of the Earth, average, m
  
  //Steering algorithm tuning parameters:
  #define HEADING_DEADBAND 2.5 //deadband +/- degrees deviation between actual and desired; within this range no servo updates will occur this loop
  
  #define DESIRED_YAW_DEADBAND 1 //yaw rate deadband +/- degrees/sec for no yaw rate adjustment in next loop (holds servo values constant)
  #define NUM_LOOPS_BEFORE_SCALING_TURN 1 //number of loops to hold steering value before stepping to a new servo pull value; value of 1 updates every loop, 2 every 2 loops, etc.
  #define FINE_SCALING_CUTOFF_DEG_SEC 2.5 //if yaw rate deviation from actual to desired (+/-) is less than this value, steering gain stepping uses fine increments
  #define FINE_SCALING_STEP_PERCENT 0.010 //range 0 to 1; servo pull percent change each update (10 times/sec) to achieve target yaw rate when yaw rate deviation < FINE_SCALING_CUTOFF_DEG_SEC
  #define FINE_SCALING_UNWIND_GAIN 1 //damping factor to increase rate of toggle line unwind; prevents underdamped overshoo of yaw rate and desired heading at low deviation angles
  
  //usage example: 1.5 means toggle line unwrap step size will be 50% greater than toggle line pull step size (150% of pull rate); 1 is inactive
  #define COARSE_SCALING_STEP_PERCENT 0.020 //range 0 to 1; servo pull percent change each update (10 times/sec) to achievetarget yaw rate when yaw rate deviation >= FINE_SCALING_CUTOFF_DEG_SEC
  #define COARSE_SCALING_UNWIND_GAIN 1.5 //damping factor to increase rate of toggle line unwind; prevents underdamped overshoot of yaw rate and desired heading at low deviation angles
  //usage example: 1.5 means toggle line unwrap step size will be 50% greater than toggle line pull step size (150% of pull rate)
  
   //Desired yaw rate coefficients; controls desired yaw rate for a given heading deviation -- get these from Excel plot of esired yaw rate vs. heading deviation
  #define DESIREDYAW_COEFF_1 -0.00000000000937 //x^6 term
  #define DESIREDYAW_COEFF_2 0.00000000662071 //x^5 term
  #define DESIREDYAW_COEFF_3 -0.00000185521159 //x^4 term
  #define DESIREDYAW_COEFF_4 0.00026074766721 //x^3 term
  #define DESIREDYAW_COEFF_5 -0.01906582376881 //x^2 term
  #define DESIREDYAW_COEFF_6 0.76467370416140 //x term
  #define DESIREDYAW_COEFF_7 -0.41502129438777 //constant term
  
  //Landing routine constants:
  #define NUM_LOOPS_BEFORE_SCALING_TURN_LANDING 1 //number of loops to hold steering value before stepping to a new servo pull value; value of 1 updates every loop, 2 every 2 loops, etc.
  #define FINE_SCALING_CUTOFF_DEG_SEC_LANDING 5 //if yaw rate deviation (+/-) is less than this value, steering gain stepping uses fine increments
  #define FINE_SCALING_STEP_PERCENT_LANDING 0.015 //range 0 to 1; servo pull percent change each update (10 times/sec) to achieve target yaw rate when yaw rate deviation < FINE_SCALING_CUTOFF_DEG_SEC
  #define COARSE_SCALING_STEP_PERCENT_LANDING 0.035 //range 0 to 1; servo pull percent change each update (10 times/sec) toachieve target yaw rate when yaw rate deviation >= FINE_SCALING_CUTOFF_DEG_SEC
  #define LANDING_RADIUS_THRESHOLD 7.5 //when within this distance from target, landingRoutine is active, m
  #define LANDING_RADIUS 6 //desired spiral radius from target, m; MUST BE LESS THAN LANDING_RADIUS_THRESHOLD!!!
  //used to calculate required yaw rate to steer within LANDING_RADIUS_THRESHOLD
  //be careful with size! smaller size requires a greater yaw rate!
  
  //Flare routine constants:
  #define FLARE_HEIGHT 4 //distance above ground to initiate flare maneuver, meters
  #define FLARE_PRCNT 1.0 //range 0 to 1; percentage of maximum servo pull used in flare maneuver
  
  //END Josh's code
  //***********************************************************************************************************************
  
  
  //********************************************************************//
  //***Debug Code Start -- comment ALL out for Monkey!***
  //
  //Used to feed fake data to control routine for compilation on
  //Windows-based PC
  
  #define DEGREES_TO_RAD 0.017453292519943295769236907684886
  #define RAD_TO_DEG 57.295779513082320876798154814105
  //#define SERVO_RIGHT_WINCH_3 2
  //#define SERVO_LEFT_WINCH_4 3
  #define SERVO_RIGHT_WINCH_3 0
  #define SERVO_LEFT_WINCH_4 1
  #define SERVO_R_WINCH_MAX_TRAVEL 5.75
  #define SERVO_L_WINCH_MAX_TRAVEL -5.75
  #define SERVO_R_WINCH_SCALE_FACTOR 1.75
  #define SERVO_L_WINCH_SCALE_FACTOR -1.75
  
  #define SCALE_FACTOR 3.0
  #define MIN_TRAVEL 58.0 // 360 degrees CCW of center
  #define CENTER_TRAVEL 90.0
  #define MAX_TRAVEL 123.0 // 360 degrees CW of center
  #define LEFT_SERVO 5
  #define RIGHT_SERVO 6
  
  //***Debug Code End***
  //********************************************************************//
  
  

#endif	/* SIMPLE_CONTROL_H */

