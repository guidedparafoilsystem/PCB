#include <Servo.h>

//********************************************************************//
//***Debug Code Start -- comment ALL out for Monkey!***
//This feeds fake data to the control routines for compilation, simulation,
//and debugging on a Windows PC
#include <math.h>
#include <stdio.h>
#include "simple_control.h"
#include <Servo.h>
#include <stdlib.h>


//GLOBALS
//***Debug Code End***

//************************************************************************************
//Start Josh's control code -- current version 3/12/2012 -- last updated 4/22/2012 44
//********************************************************************//
//Function: simple_control
//
//Desc: Control loop for flight; determines heading and distance to target
//Receives:
//Returns:
//CONSTANTS: EARTHRAD, TARGETLAT, TARGETLONG, LANDING_RADIUS_THRESHOLD,
// DEGREES_TO_RAD, RAD_TO_DEG, TARGET_ALTITUDE, FLARE_HEIGHT
//Globals: headingAct[2]

//Global variables:
float headingAct[2] = {0,0}; //actual heading, initialize {0,0} here; updated in loop
float servoOutPrcnt[2] = {0,0}; //intialize {0,0}; corrective steering command to servo without amplification scaling, a fn of headingDev
float desiredYawRate[2] = {0,0}; //intialize {0,0}; minimum desired yaw rate, a fn of headingDev, degrees/s, must be L ESS THAN 180 deg/s or steering routine WILL FAIL
int steeringActive = 0; //intialize 0; 1 if steering occured previously this loop, 0 if not
float steeringScale[2] = {0,0}; //intialize {0,0}; servo scaling factor to achieve desired yaw
float achievedYawRate[2] = {0.0}; //intialize {0,0}; yaw rate achieved by previous turn command, degrees/s
int steerDelayCounter = 0; //intialize 0; counter for initial steering input lag delay
int turnedLeft[2] = {0,0}; //intialize {0,0}; flag -- if true, system turned left last loop (but NOT as a result of negative servo value)
int turnedRight[2] = {0,0}; //intialize {0,0}; flag -- if true, system turned right last loop (but NOT as a result of negative servo value)
double headingTime[2] = {0,0}; //initialize {0,0}; holds GPS time-of-week (TOW) in milliseconds 
float gGPSheading = 0;//284;
float gGPSlatitude = 0.0;//37.2862000;
float gGPSlongitude = 0.0;//-121.8517000;
float gGPSgndspeed = 0;//15;
float gGPSaltitude = 0;//82.1550;
double gGPSTOW = 0;//579857000;

servo servo_out[2];

Servo servoL;
Servo servoR;

void init_Servos(void){
  servoL.attach(LEFT_SERVO);
  servoR.attach(RIGHT_SERVO);
  servoL.write(CENTER_TRAVEL);
  servoR.write(CENTER_TRAVEL); // 2 is minimum, via testing
  delay(1000);
  
}

void Servos_Update_All(void){
  float leftVal = ((servo_out[SERVO_LEFT_WINCH_4].value * 90.0)/SCALE_FACTOR);
  float rightVal = ((servo_out[SERVO_RIGHT_WINCH_3].value * 90.0)/SCALE_FACTOR);
  
  leftVal = leftVal/10.0;
  rightVal = rightVal/10.0; // scale properly
  
  leftVal = CENTER_TRAVEL + leftVal;
  rightVal = CENTER_TRAVEL + rightVal;
  
  // Very ugly logic to follow:
  if(leftVal >= MAX_TRAVEL) servo_out[SERVO_LEFT_WINCH_4].converted_value = MAX_TRAVEL;
  else if(leftVal <= MIN_TRAVEL) servo_out[SERVO_LEFT_WINCH_4].converted_value = MIN_TRAVEL;
  else if(leftVal < CENTER_TRAVEL) servo_out[SERVO_LEFT_WINCH_4].converted_value = (int)floor(leftVal);
  else servo_out[SERVO_LEFT_WINCH_4].converted_value = (int)ceil(leftVal);
  if(rightVal >= MAX_TRAVEL) servo_out[SERVO_RIGHT_WINCH_3].converted_value = MAX_TRAVEL;
  else if(rightVal <= MIN_TRAVEL) servo_out[SERVO_RIGHT_WINCH_3].converted_value = MIN_TRAVEL;
  else if(rightVal < CENTER_TRAVEL) servo_out[SERVO_RIGHT_WINCH_3].converted_value = (int)floor(rightVal);
  else servo_out[SERVO_RIGHT_WINCH_3].converted_value = (int)ceil(rightVal);
  
  Serial.println(servo_out[SERVO_LEFT_WINCH_4].value);
  Serial.println(servo_out[SERVO_LEFT_WINCH_4].converted_value);
  Serial.println(servo_out[SERVO_RIGHT_WINCH_3].value);
  Serial.println(servo_out[SERVO_RIGHT_WINCH_3].converted_value);
  
  servoL.write(servo_out[SERVO_LEFT_WINCH_4].converted_value);
  servoR.write(servo_out[SERVO_RIGHT_WINCH_3].converted_value);  
}

void test_servos(void){
  
  int pos = 0;
  
   for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    servoL.write(pos);               // tell servo to go to position in variable 'pos'
    servoR.write(pos); 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    servoL.write(pos);               // tell servo to go to position in variable 'pos'
    servoR.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
}

void simple_control(void)
{
  // Local variables:
  // (for arrays, 0 is current value and 1 is value from previous loop)
  // current device latitude, dec. degrees
  double latCurrent;
  // current device longitude, dec. degrees
  double longCurrent; 
  // current device altitude AGL, meters
  double altCurrent;
  // difference between target and current latitude
  double dLat;
  // difference between target and current longitude
  double dLong;
  // current pos. latitude in radians
  double latCurrentRad;
  // target pos. latitude in radians
  double targetLatRad;
  // value for Haversine distance calculation
  double aDist;
  // value for Haversine distance calculation
  double cDist;
  // variable for heading-to-target calculation
  // 'yaw
  double yHead;
  // variable for heading-to-target calculation
  double xHead;
  // desired heading to target, calculated at each loop update, degrees
  float headingDes;
  // distance magnitude from current pos. to target, m
  float distMag;
  // holds rounded desired heading to work with modulo operator
  int headingDesRnd;
  //DEBUG LINE
//  printf("x________\nStart control loop\n**************************************************************\n");

  //printf("\n______________________________________________________________\n");
  //printf("Start control loop\n**************************************************************\n");

  //Writes current lat/long in degrees from Monkey
  //latCurrent = gGPS.latitude;
  //longCurrent = gGPS.longitude;
  //altCurrent = gGPS.altitude;
  //DEBUG LINE
	
  latCurrent = gGPSlatitude;
  longCurrent = gGPSlongitude;
  altCurrent = gGPSaltitude;

  //Calculates distMag using Haversine formula
  dLat = ( TARGETLAT - latCurrent ) * DEGREES_TO_RAD;
  dLong = ( TARGETLONG - longCurrent ) * DEGREES_TO_RAD;
	
  //current latitude in radians
  latCurrentRad = latCurrent * DEGREES_TO_RAD;

  //current longitude in radians
  targetLatRad = TARGETLAT*DEGREES_TO_RAD;

  aDist = sin( dLat / 2 )	* sin( dLat / 2 )
    + sin( dLong / 2 )* sin( dLong / 2 )
    * cos( latCurrentRad ) * cos( targetLatRad );
	
  cDist = 2 * atan2( sqrt ( aDist ), sqrt( 1 - aDist ));
	
  distMag = EARTHRAD * cDist;

  //Calculates headingDes using results from above
  yHead = sin( dLong ) * cos( targetLatRad );
	
  xHead = cos( latCurrentRad ) * sin( targetLatRad )
    - sin( latCurrentRad )
    * cos(targetLatRad)*cos(dLong);
	
  //returns desired heading in degrees from -180 to 180
  headingDes = atan2( yHead, xHead ) * RAD_TO_DEG;

  //rounds headingDes for the module oeprator on next line for the modulo operator on next line
  headingDes >= 0 ? ( headingDesRnd = (int)( headingDes+0.5 ))
    : ( headingDesRnd = (int)( headingDes-0.5 ));

  //uses headingDesRnd to return 0 <= int headingDes < 360 
  headingDes = (headingDesRnd+360) % 360; 

  //DEBUG LINE

//  printf("\n Distance to target: %.3f\n Desired heading: %.3f\n Altitude: %.3f\n", distMag, headingDes, altCurrent);

  //printf("Distance to target: %.3f\n", distMag);
  //printf("Desired heading: %.3f\n", headingDes);
  //printf("Altitude: %.3f\n", altCurrent);

  // If within desired radius of target, start circling
  if (altCurrent < (TARGET_ALTITUDE + FLARE_HEIGHT))
    {
      landingFlare();
    }
	
  // If not to target yet, keep flying to it
  else if (distMag < LANDING_RADIUS_THRESHOLD)
    {
      landingRoutine(headingDes);
    }
  else
    {
      steerToTarget ( headingDes );
    }
}
//End control loop routine 


//********************************************************************//
//Function: steerToTarget
//
//Desc: Steers device proportionally as-the-crow-flies to target coords.
//
//Receives: float headingDes
//
//Returns:
//
//CONSTANTS: HEADING_DEADBAND, DESIREDYAW_COEFF[1-7], NUM_LOOPS_BEFORE_SCALING_TURN,
// DESIRED_YAW_DEADBAND, SERVO_RIGHT_WINCH_3, SERVO_LEFT_WINCH_4,
// SERVO_R_WINCH_MAX_TRAVEL, SERVO_L_WINCH_MAX_TRAVEL, SERVO_R_WINCH_SCALE_FACTOR,
// SERVO_L_WINCH_SCALE_FACTOR, FINE_SCALING_UNWIND_GAIN, COARSE_SCALING_UNWIND_GAIN,
// FINE_SCALING_STEP_PERCENT, COARSE_SCALING_STEP_PERCENT, FINE_SCALING_CUTOFF_DEG_SEC
//
//Globals: headingAct[2], servoOutPrcnt[2], desiredYawRate[2], steeringActive,
// steeringScale, achievedYawRate[2], steerDelayCounter, turnedLeft[2], turnedRight[2],
// gGPS.heading, gGPS.latitude, gGPS.longitude, gGPS.TOW, headingTime[2]
//____________________________________________________________________//

void steerToTarget(float headingDes)
{
  //Local variables:

  //deviation angle from actual heading to desired, degrees
  float headingDev;

  //loop counter for rewriting achievedYawRate array values to "older" positions in array 
  int loopCtr;

  //DEBUG LINE - comment out below for debug
  //Led_On(LED_RED); //Red LED blinks while control is in this routine (turned off at end of loop)

  //Writes current GPS heading in degrees and GPS time-of-week in milliseconds from Monkey to arrays' 0 position
  //headingAct[0] = gGPS.heading;
  //headingTime[0] = gGPS.TOW; //NOTE: using TOW for headingTime will cause a momentary glitch when TOW reverts to 0 each week

  //DEBUG LINE
  headingAct[0] = gGPSheading;
  headingTime[0] = gGPSTOW;

  //DEBUG LINES BELOW
  //printf("Actual heading: %.3f\n", headingAct[0]);
  //printf("***Start steer to target routine***\n");

  //Determines the flight heading deviation from actual to the desired heading
  //Positive clockwise, negative counterclockwise; range -180 to 180 degrees
  if ((headingDes-headingAct[0] >= -180) && (headingDes-headingAct[0] <= 180))
    {
      headingDev = headingDes-headingAct[0];
    }
  else if (headingDes-headingAct[0] < -180)
    {
      headingDev = 360 + (headingDes-headingAct[0]);
    }
  else
    {
      headingDev = (headingDes-headingAct[0]) - 360;
    }

  //Determines heading angle (yaw) change from previous loop to current loop and achievedYawRate in deg/s
  //Positive clockwise, negative counterclockwise
  //Delivers rate achieved between -180 to 180 deg/s

  //only updates achievedYawRate when a new GPS heading is available; approx. 4 times/s ec
  if ( headingAct[0] != headingAct[1] )
    {
      if (( headingAct[0] - headingAct[1] >= -180 ) && ( headingAct[0] - headingAct[1] <= 180 ))
	{
	  achievedYawRate[0] = (360 + headingAct[0]-headingAct[1])/((headingTime[0]-headingTime[1])/1000);
	}
      else if (headingAct[0]-headingAct[1] < -180)
	{
	  achievedYawRate[0] = (headingAct[0]-headingAct[1] - 360)/((headingTime[0]-headingTime[1])/1000);
	}
      else
	{
	  achievedYawRate[0] = (headingAct[0]-headingAct[1])/((headingTime[0]-headingTime[1])/1000);
	}
    }


	
  //Returns desiredYawRate = fn(headingDev) as a rate (+/- deg/s), positive clockwise
  if (headingDev<0)
    {
      desiredYawRate[0] = ( pow( fabs( headingDev ), 6 ) * DESIREDYAW_COEFF_1
			    + pow( fabs( headingDev ), 5 ) * DESIREDYAW_COEFF_2
			    + pow( fabs( headingDev ), 4 ) * DESIREDYAW_COEFF_3
			    + pow( fabs( headingDev ), 3 ) * DESIREDYAW_COEFF_4
			    + pow( fabs( headingDev ), 2 ) * DESIREDYAW_COEFF_5
			    + ( fabs( headingDev )) * DESIREDYAW_COEFF_6
			    + DESIREDYAW_COEFF_7 ) * (-1);
    }
  else
    {
      desiredYawRate[0] = ( pow( fabs( headingDev ), 6 ) * DESIREDYAW_COEFF_1
			    + pow( fabs( headingDev ), 5 ) * DESIREDYAW_COEFF_2
			    + pow( fabs( headingDev ), 4 ) * DESIREDYAW_COEFF_3
			    + pow( fabs( headingDev ), 3 ) * DESIREDYAW_COEFF_4
			    + pow( fabs( headingDev ), 2 ) *DESIREDYAW_COEFF_5
			    + ( fabs( headingDev )) * DESIREDYAW_COEFF_6
			    + DESIREDYAW_COEFF_7);
    }
  
  // Writes steering scale value of previous loop to "old" array value
  steeringScale[1] = steeringScale[0];

  // Resets steeringScale to starting value when turn direction changes as a result of crossing over desired heading,
  // or in heading deadband to prepare for next turn AFTER leaving deadband
  if ( !steeringActive || (turnedLeft[0] == 1 && headingDev > HEADING_DEADBAND) || (turnedRight[0] == 1 && headingDev < -HEADING_DEADBAND))
    {
      steeringScale[0] = 0;
      steeringScale[1] = 0;
    }

  // Writes previous turn flag values to "old" array values
  turnedLeft[1] = turnedLeft[0];
  turnedRight[1] = turnedRight[0];

  // Calculates new steeringScale value based on desired yaw rate and achieved delta yaw angle during previous control loop
  // If yaw rate is within deadband, or enough loops haven't occurred before update, steeringScale[0] is unchanged
  // If NUM_LOOPS_BEFORE_SCALING_TURN = 1, steering value is changed every loop; if = 2, changed every 2 loops, and so on
  if (steerDelayCounter % NUM_LOOPS_BEFORE_SCALING_TURN == 0)
    {
      // Don't scale unless +/- yaw rate error is greater than deadband value AND we are not in the heading deadband
      if((fabs(achievedYawRate[0]-desiredYawRate[0]) > DESIRED_YAW_DEADBAND) && (fabs(headingDev) > HEADING_DEADBAND))
	{
	  //In the case of different signs for desired yaw rate vs. achieved yaw rate, increase servo pull:
	  //For yaw rate deviation less than cutoff threshold, use FINE scaling
	  if((achievedYawRate[0] / desiredYawRate[0] < 0) && (fabs(desiredYawRate[0]-achievedYawRate[0]) < FINE_SCALING_CUTOFF_DEG_SEC))
	    { 
	      steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT;
	    }
	  //For yaw rate deviation greater than cutoff threshold, use COARSE scaling
	  else if((achievedYawRate[0] / desiredYawRate[0] < 0) && (fabs(desiredYawRate[0]-achievedYawRate[0]) >= FINE_SCALING_CUTOFF_DEG_SEC))
	    {
	      steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT;
	    }
	  //For yaw rate deviation less than cutoff threshold, use FINE scaling
	  else if(fabs(desiredYawRate[0]-achievedYawRate[0]) < FINE_SCALING_CUTOFF_DEG_SEC)
	    { 
	      //Increase yaw rate, fine stepping
	      if((achievedYawRate[0] >= 0 && desiredYawRate[0] >= 0) && (achievedYawRate[0] < desiredYawRate[0]))
		{
		  steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT;
		}
	      //Increase yaw rate, fine stepping
	      else if((achievedYawRate[0] < 0 && desiredYawRate[0] < 0) && (achievedYawRate[0] >= desiredYawRate[0]))
		{
		  steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT;
		}
	      else
		{
		  //Decrease yaw rate, fine stepping
		  steeringScale[0] = steeringScale[1] - (FINE_SCALING_STEP_PERCENT * FINE_SCALING_UNWIND_GAIN);
		}
	    }
	  //For yaw rate deviation more than cutoff threshold, use COARSE scaling
	  else if(fabs(desiredYawRate[0]-achievedYawRate[0]) >= FINE_SCALING_CUTOFF_DEG_SEC)
	    {
	      //Increase yaw rate, coarse stepping
	      if((achievedYawRate[0] >= 0 && desiredYawRate[0] >= 0) && (achievedYawRate[0] < desiredYawRate[0]))
		{ 
		  steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT;
		}
	      //Increase yaw rate, coarse stepping
	      else if((achievedYawRate[0] < 0 && desiredYawRate[0] < 0) && (achievedYawRate[0] >= desiredYawRate[0]))
		{
		  steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT;
		}
	      //Decrease yaw rate, coarse stepping
	      else
		{
		  steeringScale[0] = steeringScale[1] - (COARSE_SCALING_STEP_PERCENT * COARSE_SCALING_UNWIND_GAIN);
		}
	    }
	}
    }

  //Keeps the system from driving servos past 100% pull
  if (steeringScale[0] > 1)
    {
      steeringScale[0] = 1;
    }
  
  if (steeringScale[0] < -1)
   {
     steeringScale[0] = -1;
   }
  

  //  ????? where does this go ???? : In the case of desired and achieved yaw rate both positive or both negative:

  //servoOutPrcnt is the fraction of maximum servo pull for turning, 0 to 1
  servoOutPrcnt[0] = steeringScale[0];

  //DEBUG LINE
  //printf("\nThis loop's steering commands:\n steeringScale[0,1]: [%.6f, %6f]\n servoOutPrcnt[0,1]: [%.6f, %.6f]\n",
	//	 steeringScale[0], steeringScale[1], servoOutPrcnt[0], servoOutPrcnt[1]);

  //Writes current pre-turn values to previous values for next loop
  servoOutPrcnt[1] = servoOutPrcnt[0];

  //Determines which direction to turn and commands the servos to steer, scaled by steeringScale
  //If steeringScale=1, full deflection is commanded (5.75 inch toggle line pull on parafoil)
  if ((headingDev < -HEADING_DEADBAND && servoOutPrcnt[0] >= 0) || (headingDev > HEADING_DEADBAND && servoOutPrcnt[0] < 0 ))
    {
      //then turn left!
      servo_out[ SERVO_LEFT_WINCH_4 ].value = servo_out[ SERVO_LEFT_WINCH_4 ].value_failsafe
	- ( fabs( servoOutPrcnt[0] ) * SERVO_L_WINCH_MAX_TRAVEL * SERVO_L_WINCH_SCALE_FACTOR );

	  //printf("servo_out: %f", servo_out[ SERVO_LEFT_WINCH_4 ].value);
	  
      servo_out[ SERVO_RIGHT_WINCH_3 ].value = servo_out[ SERVO_RIGHT_WINCH_3 ].value_failsafe;
      //For DEBUG, disable next line
      Servos_Update_All();

      // flag set for left turn, but ONLY for a turn not caused negative servo value
      if (headingDev < -HEADING_DEADBAND && servoOutPrcnt[0] >= 0)
	{
	  turnedLeft[0] = 1;
	  turnedRight[0] = 0;
	}
      //DEBUG LINE BELOW
      //printf("\nSteering Left\n");
    }
  //then turn right!
  else if ((headingDev > HEADING_DEADBAND && servoOutPrcnt[0] >= 0) || (headingDev < -HEADING_DEADBAND && servoOutPrcnt[0 ] < 0))
    {
      servo_out[ SERVO_RIGHT_WINCH_3 ].value = servo_out[ SERVO_RIGHT_WINCH_3 ].value_failsafe
	+ (fabs(servoOutPrcnt[0])*SERVO_R_WINCH_MAX_TRAVEL*SERVO_R_WINCH_SCALE_FACTOR);
      
      
      servo_out[ SERVO_LEFT_WINCH_4 ].value = servo_out[ SERVO_LEFT_WINCH_4 ].value_failsafe;
      //For DEBUG, disable next line
      Servos_Update_All();
      
      // flag set for left turn, but ONLY for a turn not caused by negative servo value
      if (headingDev > HEADING_DEADBAND && servoOutPrcnt[0] >= 0)
	{
	  turnedRight[0] = 1;
	  turnedLeft[0] = 0;
	}
	      
      //DEBUG LINE BELOW
      //printf("\nSteering Right\n"); 
    }
  else
    {
      turnedLeft[0] = 0;
      turnedRight[0] = 0;

      //DEBUG LINE BELOW
      //printf("\nServos Unchanged -- In Heading Deadband\n"); 	      
    }

  //	} // ************** ADDED MISSING CLOSE PARENS? ************************//

  
  //Sets steeringActive flag true if steering occurred earlier this loop and
  //disables the reset of steering scale to 1 until straight flight resumes
  if (headingDev > -HEADING_DEADBAND && headingDev < HEADING_DEADBAND)
    {
      steeringActive = 0;
    }
  else
    {
      steeringActive = 1; 
    }
  

      //Counter for how many loops occur at minimum steer before steer output begins scaling //Intended to account for delay in heading change after initial steering input
  if (steeringActive)
    {
      steerDelayCounter+= 1;
    }
  else
    {
      steerDelayCounter = 0;
    }
  
  
  //DEBUG LINE
  //printf("\nValues for this loop:\n steerDelayCounter (value for next loop): %d\n headingDev from actual-->desired: %.3f\n"
	//	 " achievedYawRate[0] (prev. loop to now): %.3f\n"
	//	 " desiredYawRate[0,1]: [%.3f, %.3f]\n steeringActive: %d\n",
	//	 steerDelayCounter, headingDev, achievedYawRate[0], desiredYawRate[0], desiredYawRate[1], steeringActive );
  
  //Writes current values to previous values for next loop
  desiredYawRate[1] = desiredYawRate[0];
  headingAct[1] = headingAct[0];
  headingTime[1] = headingTime[0];
  achievedYawRate[1] = achievedYawRate[0];
  
  //DEBUG LINE - comment out below for debug
  //Led_Off(LED_RED); //Red LED blinks while control is in this routine (turned on at beginning of loop)
}
    

 //********************************************************************//
 //Function: landingRoutine
 //
 //Desc: Steers device proportionally in a circle around the target coordinate,
 // with radius <= LANDING_RADIUS_THRESHOLD
 //
 //Receives: float headingDes
 //
 //Returns:
 //
 //CONSTANTS: LANDING_RADIUS_THRESHOLD, HEADING_DEADBAND,
 // NUM_LOOPS_BEFORE_SCALING_TURN_LANDING, DESIRED_YAW_DEADBAND,
 // LANDING_RADIUS, DEGREES_TO_RAD, SERVO_RIGHT_WINCH_3, SERVO_LEFT_WINCH_4,
 // SERVO_R_WINCH_MAX_TRAVEL, SERVO_L_WINCH_MAX_TRAVEL, SERVO_R_WINCH_SCALE_FACTOR,
 // SERVO_L_WINCH_SCALE_FACTOR, FINE_SCALING_UNWIND_GAIN, COARSE_SCALING_UNWIND_GAIN
 //
 //Globals: headingAct[2], servoOutPrcnt[2], desiredYawRate[2], steeringActive,
 // steeringScale, achievedYawRate[2], steerDelayCounter, turnedLeft[2], turnedRight[2],
 // gGPS.heading, gGPS.latitude, gGPS.longitude, gGPS.gndspeed, gGPS.TOW, headingTime[2]
 //____________________________________________________________________//


void landingRoutine(float headingDes)
{


  //Local variables:
  //deviation angle from actual heading to desired, degrees
  float headingDev;
  //loop counter for rewriting achievedYawRate array values to "older" positions in array
  int loopCtr; 

  //DEBUG LINE - comment out below for debug
  //Led_On(LED_RED); //Red LED is on steady in this routine
  
  //Writes current GPS heading in degrees and GPS time-of-week in milliseconds from Monkey to arrays' 0 position
  //headingAct[0] = gGPS.heading;
  //headingTime[0] = gGPS.TOW; //NOTE: using TOW for headingTime will cause a momentary glitch when TOW reverts to 0 each week
  //DEBUG LINE
  headingAct[0] = gGPSheading;
  headingTime[0] = gGPSTOW;
  
  //DEBUG LINES BELOW
 //printf(" Actual heading: %.3f\n", headingAct[0]);
 //printf("\n\n***Start landing routine***\n");

 //Determines the flight heading deviation from actual to the desired heading assuming straight flight to determine required landing spiral direction
 //Positive clockwise, negative counterclockwise; range -180 to 180 degrees
 if ((headingDes-headingAct[0] >= -180) && (headingDes-headingAct[0] <= 180))
   {
     headingDev = headingDes-headingAct[0];
   }
 else if (headingDes-headingAct[0] < -180)
   {
     headingDev = 360 + (headingDes-headingAct[0]);
   }
 else
   {
     headingDev = (headingDes-headingAct[0]) - 360;
   }
 
 //Determines new desired heading tangent to a radius around target, and the direction of spiral depending on target being approached from right or left side

 //If true, we are approaching target to the right, and should spiral in a left/CCW pattern 
 if (headingDev < 0)
   {
     //Causes parafoil to fly tangent to a CCW circle around target
     headingDes = headingDes + 90;
   }
 //If true, we are approaching target to the left, and should fly in a right/CW pattern
 else if (headingDev >= 0)
   {
     headingDes = headingDes - 90;
   }

 
 //Now re-determine headingDev for the new spiral-flight desired heading
 //Determines the flight heading deviation from actual to the desired heading for landing spiral flight path
 //Positive clockwise, negative counterclockwise; range -180 to 180 degrees
 if ((headingDes-headingAct[0] >= -180) && (headingDes-headingAct[0] <= 180))
   {
     headingDev = headingDes-headingAct[0];
   }

 else if (headingDes-headingAct[0] < -180)
   {
     headingDev = 360 + (headingDes-headingAct[0]);
   }
 else
   {
     headingDev = (headingDes-headingAct[0]) - 360;
   }

 
//DEBUG LINE
//printf("\n Desired heading for landing spiral: %.3f\n", headingDes);
 
//Determines heading angle (yaw) change from previous loop to current loop and achievedYawRate in deg/s
//Positive clockwise, negative counterclockwise
//Delivers rate achieved between -180 to 180 deg/s
//only updates achievedYawRate when a new GPS heading is available; approx. 4 times/s ec
if (headingAct[0] != headingAct[1])
  {

    if ((headingAct[0]-headingAct[1] >= -180) && (headingAct[0]-headingAct[1] <= 180))
      {
	achievedYawRate[0] = (headingAct[0]-headingAct[1])/((headingTime[0]-headingTime[1])/1000);
      }
    else if (headingAct[0]-headingAct[1] < -180)
      {
	achievedYawRate[0] = (360 + headingAct[0]-headingAct[1])/((headingTime[0]-headingTime[1])/1000);
      }
    else
      {
	achievedYawRate[0] = (headingAct[0]-headingAct[1] - 360)/((headingTime[0]-headingTime[1])/1000);
      }
  }

 //Returns desiredYawRate = fn(headingDev, LANDING_RADIUS, gGPS.gndspeed) as a rate (+/- deg/s), positive clockwise
 //Calculates the yaw rate required to maintain a circular flight path of radius LANDING_RADIUS


 //desiredYawRate[0]= (headingDev/fabs(headingDev) * gGPS.gndspeed / LANDING_RADIUS) * RAD_TO_DEG;
 //DEBUG LINE -- make sure to activate above line for Monkey!
 desiredYawRate[0]= (headingDev/fabs(headingDev) * gGPSgndspeed / LANDING_RADIUS) * RAD_TO_DEG;

 //Prevents desired yaw rate from exceeding 180 deg/s, plus some margin. Steering will fail if actual
 //yaw rate exceeds 180 deg/s.
 if (desiredYawRate[0] > 170)
   {
     desiredYawRate[0] = 170;
   }
 else if (desiredYawRate[0] < -170)
   {
     desiredYawRate[0] = -170;
   }

 //Writes steering scale value of previous loop to "old" array value
 steeringScale[1] = steeringScale[0];
 
 // Resets steeringScale to starting value when turn direction changes as a result of crossing over desired heading,
 // or in heading deadband to prepare for next turn AFTER leaving deadband
 if (!steeringActive || (turnedLeft[0] == 1 && headingDev > HEADING_DEADBAND) || (turnedRight[0] == 1 && headingDev < -HEADING_DEADBAND))
   {
     steeringScale[0] = 0;
     steeringScale[1] = 0;
   }

 //Writes previous turn flag values to "old" array values
 turnedLeft[1] = turnedLeft[0];
 turnedRight[1] = turnedRight[0];
 
 //Calculates new steeringScale valfue based on desired yaw rate and achieved delta yaw angle during previous control loop
 //If yaw rate is within deadband, or enough loops haven't occurred before update, steeringScale[0] is unchanged

 //If NUM_LOOPS_BEFORE_SCALING_TURN_LANDING = 1, ste ering value is changed every loop; if = 2, changed every 2 loops, and so on
 if (steerDelayCounter % NUM_LOOPS_BEFORE_SCALING_TURN_LANDING == 0)
   { 
     //Don't scale unless +/- yaw rate error is greater than deadband value AND we are not in the heading deadband
     if((fabs(achievedYawRate[0]-desiredYawRate[0]) > DESIRED_YAW_DEADBAND) && (fabs(headingDev) > HEADING_DEADBAND))
       { 
	 //In the case of different signs for desired yaw rate vs. achieved yaw rate, increase servo pull:
	 //For yaw rate deviation less than cutoff threshold, use FINE scaling
	 if((achievedYawRate[0] / desiredYawRate[0] < 0) && (fabs(desiredYawRate[0]-achievedYawRate[0]) < FINE_SCALING_CUTOFF_DEG_SEC_LANDING))
	   {
	     steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT_LANDING;
	   }
	 //For yaw rate deviation greater than cutoff threshold, use coarse scaling 
	 else if((achievedYawRate[0] / desiredYawRate[0] < 0) && (fabs(desiredYawRate[0]-achievedYawRate[0]) >= FINE_SCALING_CUTOFF_DEG_SEC_LANDING))
	   {
	     steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT_LANDING;
	   }
	 //In the case of desired and achieved yaw rate both positive or both negative:
	 //For yaw rate deviation less than cutoff threshold, use FINE scaling
	 else if(fabs(desiredYawRate[0]-achievedYawRate[0]) < FINE_SCALING_CUTOFF_DEG_SEC_LANDING)
	   { 
	     //Increase yaw rate, fine stepping
	     if((achievedYawRate[0] >= 0 && desiredYawRate[0] >= 0) && (achievedYawRate[0] < desiredYawRate[0]))
	       {
		 steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT_LANDING;
	       }
	     //Increase yaw rate, fine stepping
	     else if((achievedYawRate[0] < 0 && desiredYawRate[0] < 0) && (achievedYawRate[0] >= desiredYawRate[0]))
	       {
		 steeringScale[0] = steeringScale[1] + FINE_SCALING_STEP_PERCENT_LANDING;
	       }
	     //Decrease yaw rate, fine stepping
	     else
	       {
		 steeringScale[0] = steeringScale[1] - (FINE_SCALING_STEP_PERCENT_LANDING * FINE_SCALING_UNWIND_GAIN);
	       }
	   }
	 //For yaw rate deviat ion more than cutoff threshold, use COARSE scaling
	 else if(fabs(desiredYawRate[0]-achievedYawRate[0]) >= FINE_SCALING_CUTOFF_DEG_SEC_LANDING)
	   {
	     //Increase yaw rate, coarse stepping
	     if((achievedYawRate[0] >= 0 && desiredYawRate[0] >= 0) && (achievedYawRate[0] < desiredYawRate[0]))
	       {
		 steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT_LANDING;
	       }
	     //Increase yaw rate, coarse stepping
	     else if((achievedYawRate[0] < 0 && desiredYawRate[0] < 0) && (achievedYawRate[0] >= desiredYawRate[0]))
	       {
		 steeringScale[0] = steeringScale[1] + COARSE_SCALING_STEP_PERCENT_LANDING;
	       }
	     else
	       {
		 //Decrease yaw rate, coarse stepping
		 steeringScale[0] = steeringScale[1] - (COARSE_SCALING_STEP_PERCENT_LANDING * COARSE_SCALING_UNWIND_GAIN);
	       }
	   }
       }
   }
 
 //Keeps the system from driving servos past 100% pull 
 if (steeringScale[0] > 1)
   {
     steeringScale[0] = 1;
   }

 if (steeringScale[0] < -1)
   {
     steeringScale[0] = -1;
   }

 //servoOutPrcnt is the fraction of maximum servo pull for turning, 0 to 1
 servoOutPrcnt[0]=steeringScale[0];


 //DEBUG LINE
 //printf("\nThis loop's steering commands:\n steeringScale[0,1]: [%.6f, %6f]\n servoOutPrcnt[0,1]: [%.6f, %.6f]\n", steeringScale[0], steeringScale[1], servoOutPrcnt[0], servoOutPrcnt[1]); 

 //Writes current pre-turn values to previous values for next loop
 servoOutPrcnt[1] = servoOutPrcnt[0];



 //Determines which direction to turn and commands the servos to steer, scaled by steeringScale
 //If steeringScale=1, full deflection is commanded (5.75 inch toggle line pull on parafoil)

 //then turn left!
 if ((headingDev < -HEADING_DEADBAND && servoOutPrcnt[0] >= 0) || (headingDev > HEADING_DEADBAND && servoOutPrcnt[0] < 0 ))

   {
     servo_out[ SERVO_LEFT_WINCH_4 ].value = servo_out[ SERVO_LEFT_WINCH_4 ].value_failsafe
       - (fabs(servoOutPrcnt[0])*SERVO_L_WINCH_MAX_TRAVEL*SERVO_L_WINCH_SCALE_FACTOR);

     servo_out[ SERVO_RIGHT_WINCH_3 ].value = servo_out[ SERVO_RIGHT_WINCH_3 ].value_failsafe;
     //For DEBUG, disable next line
     Servos_Update_All();

     if (headingDev < -HEADING_DEADBAND && servoOutPrcnt[0] >= 0)
       //flag set for left turn, but ONLY for a turn not cause d by negative servo value
       {
	 
	 turnedLeft[0] = 1;
	 turnedRight[0] = 0;
       }
     
     //DEBUG LINE BELOW
     //printf("\nSteering Left\n");
   }

 //then turn right!
 else if ((headingDev > HEADING_DEADBAND && servoOutPrcnt[0] >= 0) || (headingDev < -HEADING_DEADBAND && servoOutPrcnt[0 ] < 0)) 
   {
     servo_out[ SERVO_RIGHT_WINCH_3 ].value = servo_out[ SERVO_RIGHT_WINCH_3 ].value_failsafe
       + (fabs(servoOutPrcnt[0])*SERVO_R_WINCH_MAX_TRAVEL*SERVO_R_WINCH_SCALE_FACTOR);

     servo_out[ SERVO_LEFT_WINCH_4 ].value = servo_out[ SERVO_LEFT_WINCH_4 ].value_failsafe;
     //For DEBUG, disable next line
     Servos_Update_All();

     //flag set for left turn, but ONLY for a turn not caused by negative servo value
     if (headingDev > HEADING_DEADBAND && servoOutPrcnt[0] >= 0)
       {

	 turnedRight[0] = 1;
	 turnedLeft[0] = 0;
       }
     //DEBUG LINE BELOW
     //printf("\nSteering Right\n"); 
   }
 else
   {
     turnedLeft[0] = 0;
     turnedRight[0] = 0;
     
     //DEBUG LINE BELOW
//printf("\nServos Unchanged -- In Heading Deadband\n");
   }

 //Sets steeringActive flag true if steering occurred earlier this loop and
 //disables the reset of steering scale to 1 until straight flight resumes
 if (headingDev > -HEADING_DEADBAND && headingDev < HEADING_DEADBAND)
   {
     steeringActive = 0;
   }
 else
   {
  steeringActive = 1;
   }
 
 //Counter for how many loops occur at minimum steer before steer output begins scaling
 //Intended to account for delay in heading change after initial steering input
 if (steeringActive)
   {
     steerDelayCounter += 1;
   }
 
 else
   {
     steerDelayCounter = 0;
  }


 //DEBUG LINE
 //printf("\n\nValues for this loop:\n steerDelayCounter (value for next loop): %d\n headingDev from actual-->desired: %.3f\
//n achievedYawRate[0] (prev. loop to now): %.3f\n"
 //" desiredYawRate[0,1]: [%.3f, %.3f]\n steeringActive: %d\n",
 //steerDelayCounter, headingDev, achievedYawRate[0], desiredYawRate[0], desiredYawRate[1], steeringActive);

 //Writes current values to previous values for next loop
 desiredYawRate[1] = desiredYawRate[0];
 headingAct[1] = headingAct[0];
 headingTime[1] = headingTime[0];
 achievedYawRate[1] = achievedYawRate[0];

}

 //********************************************************************//
 //Function: landingFlare
 //
 //Desc: Performs a simple flare maneuver when parafoil system altitude
 // is less than FLARE_HEIGHT above ground
 //
 //Receives:
 //
 //Returns:
 //
 //CONSTANTS: SERVO_RIGHT_WINCH_3, SERVO_LEFT_WINCH_4, SERVO_R_WINCH_MAX_TRAVEL,
 // SERVO_L_WINCH_MAX_TRAVEL, SERVO_R_WINCH_SCALE_FACTOR,
 // SERVO_L_WINCH_SCALE_FACTOR, FLARE_PRCNT
 //
 //Globals:
 //____________________________________________________________________//


void landingFlare(void){
  //Sets both servos to a percentage of maximum pull to perform braking flare near landing
  servo_out[ SERVO_RIGHT_WINCH_3 ].value = servo_out[ SERVO_RIGHT_WINCH_3 ].value_failsafe
    + (FLARE_PRCNT*SERVO_R_WINCH_MAX_TRAVEL*SERVO_R_WINCH_SCALE_FACTOR);

  servo_out[ SERVO_LEFT_WINCH_4 ].value = servo_out[ SERVO_LEFT_WINCH_4 ].value_failsafe
    - (FLARE_PRCNT*SERVO_L_WINCH_MAX_TRAVEL*SERVO_L_WINCH_SCALE_FACTOR);

  //For DEBUG, disable next line
  Servos_Update_All();

  //DEBUG LINE
  //printf("\n\n***Initiate landing flare***\n Left Servo Value: %.3f\n Right Servo Value: %.3f\n", FLARE_PRCNT, FLARE_PRCNT);
}

// Receives String latitude in format <degrees[(-90) to 90].min[2]sec[6]>; converts to decimal degrees
float convert_latitude_to_decimal_degrees(String latitude)
{
  float converted_latitude;
  
 
  String deg; //[0-2]
  String secs; //[3-]
  
  deg = latitude.substring(0,2);
  latitude.remove( 0, 2 );
  
  secs = latitude;

  converted_latitude = deg.toFloat() + (secs.toFloat() / 60.0 );
  
  //Serial.print("Converted: ");
  //Serial.println(converted_latitude);
 
  //Serial.print("Degrees: ");
  //Serial.println(deg);
  
  //Serial.print("Rest: ");
  //Serial.println(secs);

  return converted_latitude;
}


// Receives String longitude in format <degrees[(-180) to 180].min[2]sec[6]>; converts to decimal degrees
float convert_longitude_to_decimal_degrees(String longitude)
{
  float converted_longitude;
   
  String deg; //[0-2]
  String secs; //[3-]
  char neg = '-';
  
  if(longitude.charAt(0) == neg)
  {
    // Serial.println("IS NEGATIVE");
    longitude.remove(0,1);
  }
  
  deg = longitude.substring(0,3);
  longitude.remove( 0, 3 );
  
  secs = longitude;

  converted_longitude = deg.toFloat() + (secs.toFloat() / 60.0 );
  
  //Serial.print("Converted: ");
  //Serial.println(converted_longitude);
 
  //Serial.print("Degrees: ");
  //Serial.println(deg);
  
  //Serial.print("Rest: ");
  //Serial.println(secs);
  
  return -1 * converted_longitude;
}

void flight_control(String gpsData) // Written by Richard Park
{
  float gps_latitude;
  float gps_longitude;
  float gps_altitude;
  
  String string_lat;
  String string_long;
  String string_alt;
  
  static float prev_alt = 0;
  
  String s = gpsData;
  String delimiter = ",";
  String token;
  
  bool enable_control = false;
  
  int pos = 0;
  int count = 0;
  int comma_count = 0;
  static int j = 0; // counter
          
  // Arbitrarily large array to store list;
  String token_list[20];
  
  // Parse, tokenize and store in array 
  while ( comma_count != 15 )
  {
                  comma_count++;
                  pos = s.indexOf(",");
                  token = s.substring(0, pos);
  token_list[ count ] = token;
  
  // 'Pops' item from front
  s.remove( 0, pos + delimiter.length() );
                      
  count++;
  }
  
  // Grab the desired tokens
  string_lat = token_list[2];
  string_long = token_list[4];
  string_alt = token_list[9];
  
  // Sanity checks; if NULL; no update!
  if (string_lat.length() != 0){
    gGPSlatitude = gps_latitude = convert_latitude_to_decimal_degrees(string_lat);
  }
  
  if (string_long.length() != 0){
    gGPSlongitude = gps_longitude = convert_longitude_to_decimal_degrees(string_long);
  }
  
  if (string_alt.length() != 0){
    gGPSaltitude = gps_altitude = string_alt.toFloat();
  }
  
  //gGPSlatitude = gps_latitude; // update global variables
  //gGPSlongitude = gps_longitude;
  
  Serial.println("Fixed lat: " + String(gGPSlatitude,DEC));
  //gGPSaltitude = gps_altitude;
  
  Serial.println(gps_latitude,5);
  Serial.println(gps_longitude,5);
  Serial.println(gps_altitude,5);
  
  if((prev_alt > gGPSaltitude) && (millis() > 2000000)){ // 30000 seconds
    if(j > 3){
      simple_control();
      enable_control = true;
    }
    j++;
  }
  else j = 0;
  
  if(enable_control){
    simple_control();
  }
  
  prev_alt = gps_altitude; // store previous altitude
}
//End Josh's control code
