#include <ax12.h>
#include "scottoLeg.h"
#include "scottoMotorInterface.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>

File myFile;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
scottoLeg Legs[4];

// leg / motor properties
int leg0[] = {1,2,3};//front left
int leg1[] = {7,8,9};//back left
int leg2[] = {12,11,10};//front right
int leg3[] = {4,5,6};//back right

//time used to track position in trajectory for each leg.
uint16_t legTime[4];

// min/maxes of all motors
int motorMins[] = {850,859,865,159,864,156,860,157,861,863,861,861}; // 156 -> 861 for motor 12 (backwards)
int motorMaxs[] = {159,159,159,869,158,860,156,863,160,157,156,156}; // NOTE: swapped min and max for 4, 5, 

float stowPos[] = {90,65,81};
//float stowPos[] = {90,35,30};
bool rightSideUp;
int rollStep = 0;
int sitStep = 0;
int standStep = 0;
int stowStep = 0;
int rollCount = 0;
int routineState = 0;
uint16_t standTime = 0;
uint16_t sitTime = 0;
uint16_t stowTime = 0;
uint16_t routineTime = 0;

bool standCommanded = true;
bool sitCommanded = true;
bool walkCommanded = true;

//Time of the gait
uint16_t gaitPeriod = 1500;
//Trajectory of the three motors on each leg
float motor_1Front[] = {42,42,41,41,40,39,38,37,35,34,33,31,30,28,27,24,23,21,19,17,15,13,11,9,6,3,0,-1,-2,-2,-1.2497,1.2221,5.2152,10.4061,16.3742,22.636,28.6843,34.0291,38.2373,40.968};
float motor_2Front[] = {-27,-27,-27,-27,-27,-28,-27,-27,-27,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-20.7816,-15.0265,-10.2009,-6.6958,-4.7951,-4.6528,-6.2804,-9.5461,-14.1853,-19.8221};
float motor_3Front[] = {-41,-41,-43,-44,-46,-47,-48,-50,-52,-53,-55,-57,-58,-60,-61,-62,-63,-64,-64,-65,-65,-66,-66,-66,-67,-67,-67,-67,-67,-67,-67,-64.1111,-61.2222,-58.3333,-55.4444,-52.5556,-49.6667,-46.7778,-43.8889,-41};
float motor_1Back[] = {-40.968,-38.2373,-34.0291,-28.6843,-22.636,-16.3742,-10.4061,-5.2152,-1.2221,1.2497,2,2,1,0,-3,-6,-9,-11,-13,-15,-17,-19,-21,-23,-24,-27,-28,-30,-31,-33,-34,-35,-37,-38,-39,-40,-41,-41,-42,-42};
float motor_2Back[] = {-19.8221,-14.1853,-9.5461,-6.2804,-4.6528,-4.7951,-6.6958,-10.2009,-15.0265,-20.7816,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-27,-27,-27,-28,-27,-27,-27,-27,-27};
float motor_3Back[] = {-41,-43.8889,-46.7778,-49.6667,-52.5556,-55.4444,-58.3333,-61.2222,-64.1111,-67,-67,-67,-67,-67,-67,-67,-66,-66,-66,-65,-65,-64,-64,-63,-62,-61,-60,-58,-57,-55,-53,-52,-50,-48,-47,-46,-44,-43,-41,-41};
float timeBetweenGaitPos;

// angles for standing position
float frontM1Goal = 0;
float backM1Goal = 0;
float m2Goal1 = 0;
float m2Goal2 = 0;
float m3Goal = 0;

void setup(){

   //open serial port
   Serial.begin(9600);
  
    // set up sd card
   Serial.print("Initializing SD card...");

  if (!SD.begin(4)) 
  {
    Serial.println("SD initialization failed!");
    while (1);
  }
  Serial.println("SD initialization done.");
  
   // Set up IMU
   Serial.println("\n Starting IMU \n");
   while(!bno.begin())
     Serial.println(".");
   bno.setExtCrystalUse(true);
   sensors_event_t event;
   bno.getEvent(&event);
   delay(100);
   rightSideUp = orientationCheck;
   
   //Initialize all motors on all legs
   Legs[0] = scottoLeg(leg0[0],leg0[1],leg0[2],motorMins[leg0[0]-1],motorMaxs[leg0[0]-1],motorMins[leg0[1]-1],motorMaxs[leg0[1]-1],motorMins[leg0[2]-1],motorMaxs[leg0[2]-1]);
   Legs[1] = scottoLeg(leg1[0],leg1[1],leg1[2],motorMins[leg1[0]-1],motorMaxs[leg1[0]-1],motorMins[leg1[1]-1],motorMaxs[leg1[1]-1],motorMins[leg1[2]-1],motorMaxs[leg1[2]-1]);
   Legs[2] = scottoLeg(leg2[0],leg2[1],leg2[2],motorMins[leg2[0]-1],motorMaxs[leg2[0]-1],motorMins[leg2[1]-1],motorMaxs[leg2[1]-1],motorMins[leg2[2]-1],motorMaxs[leg2[2]-1]);
   Legs[3] = scottoLeg(leg3[0],leg3[1],leg3[2],motorMins[leg3[0]-1],motorMaxs[leg3[0]-1],motorMins[leg3[1]-1],motorMaxs[leg3[1]-1],motorMins[leg3[2]-1],motorMaxs[leg3[2]-1]);
   
   // Stagger the time that the legs start at in the gait cycle
   legTime[0] = 0;
   legTime[2] = gaitPeriod/4;
   legTime[1] = gaitPeriod/2;
   legTime[3] = gaitPeriod*3/4;
   
   // Compute the time between each position in the trajectory
   timeBetweenGaitPos = (float)gaitPeriod/((float)(sizeof(motor_1Front)/sizeof(motor_1Front[0])-1));
   
   // compute angles for standing position
  for (int i = 0; i < (sizeof(motor_1Front)/sizeof(motor_1Front[0]));i++)
  {
    frontM1Goal += motor_1Front[i];
    backM1Goal += motor_1Back[i];
    m2Goal2 = min(m2Goal2,motor_2Front[i]);
    m2Goal1 = max(m2Goal1,motor_2Front[i]);
    m3Goal = min(m3Goal,motor_3Front[i]);
    m2Goal1 = max(m2Goal1,motor_2Back[i]);
    m2Goal2 = min(m2Goal2,motor_2Back[i]);
    m3Goal = min(m3Goal,motor_3Back[i]);
  }
  frontM1Goal = frontM1Goal/(float)(sizeof(motor_1Front)/sizeof(motor_1Front[0]));
  backM1Goal = backM1Goal/(float)(sizeof(motor_1Front)/sizeof(motor_1Front[0]));
   
   
   Serial.println("\nStarting in 2 seconds");
   delay(2000);
   Serial.println("starting...");
   //stand();
   //delay(1000);
   // sit();
   //delay(2000);
   //frontLegRoll();
   delay(1000);
   //Legs[0].m3.moveSpeedDegree(-90,1000);
   //bellyRoll();
//   delay(5000);
  while (!sitTest());
  delay(1000);
  while (!stowTest());
  delay(1000);
  /* while (!standTest());
  delay(3000); */
  routineTime = millis(); // start of routine
 }

void loop(){  
  if (bellyRoll()) {
    rollCount++;
  }
  
  /*
  if (millis() - routineTime > 14000) {
    if (bellyRoll()) {
      rollCount++;
      Serial.print("rollCount: ");
      Serial.println(rollCount);
    }
    if (rollCount > 4) {
      standTest(); // stop rolling
      while(1);
    }
  }
  else if (millis() - routineTime > 14000) {
    if (bellyRoll())
      rollCount++;
    if (rollCount > 4)
      standTest(); // stop rolling
  }
  else if (millis() - routineTime > 12000) {
    Serial.println("stowing");
    stowTest();
  }
  else if (millis() - routineTime > 10000) {
    Serial.println("sitting");
    sitTest();
  }
  else{
    Serial.println("walking");
    walk();
  }
 
  */
  
  // test sitting
//  if (sitCommanded) {
//    sitStowTest();
//    Serial.println("sitting...");
//  }
//  else {
//    Serial.println("not sitting!");
//  }

  /*if (rightSideUp) // If the robot is right side up, do a belly roll
  {
    if (bellyRoll())// If the roll has finished, count the roll
      rollCount++;
  }
  else // If the robot is on its back, do a back roll
  {
    if (backRoll())// If the roll has finished, count the roll
    {
      rollCount++;
    }
  }*/
//  if (bellyRoll())
//    rollCount++;
//  if (rollCount > 4)
//    while(true);

  /*
  // Move each leg
  for (int i = 0; i < 4; i+=2)
    moveFrontLeg(Legs[i],legTime[i]);
  for (int i = 1; i < 4; i+=2)
    moveBackLeg(Legs[i],legTime[i]);
 
 // Increment each leg's time in the gait cycle 
  int incrementTime = millis() - startTime;
  for (int i = 0; i < 4; i ++)
    legTime[i] = (legTime[i]+incrementTime)%gaitPeriod;*/
  
}

// Function to move a front leg to the position given a time in a gait cycle
void moveFrontLeg(scottoLeg leg2move,uint16_t gaitTime)
{
  int gaitNum = floor((float)gaitTime/timeBetweenGaitPos); // Find the 
  float command1 = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_1Front[gaitNum],motor_1Front[gaitNum+1]);
  float command2 = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_2Front[gaitNum],motor_2Front[gaitNum+1]);
  float command3 = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_3Front[gaitNum],motor_3Front[gaitNum+1]);
  leg2move.moveAllDegree(command1,command2,command3);
}

// Function to move a back leg to the position given a time in a gait cycle
void moveBackLeg(scottoLeg leg2move,uint16_t gaitTime)
{
  int gaitNum = floor((float)gaitTime/timeBetweenGaitPos);
  float command1 = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_1Back[gaitNum],motor_1Back[gaitNum+1]);
  float command2 = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_2Back[gaitNum],motor_2Back[gaitNum+1]);
  float command3 = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_3Back[gaitNum],motor_3Back[gaitNum+1]);
  leg2move.moveAllDegree(command1,command2,command3);
}

void walk() {
  uint16_t startTime = millis();

  // Move each leg
    for (int i = 0; i < 4; i+=2) {
      moveFrontLeg(Legs[i],legTime[i]);
    }
    for (int i = 1; i < 4; i+=2) {
      moveBackLeg(Legs[i],legTime[i]);
    }
   
   // Increment each leg's time in the gait cycle 
    int incrementTime = millis() - startTime;
    Serial.print("incrementTime: ");
    Serial.println(incrementTime);
    for (int i = 0; i < 4; i ++) {
      legTime[i] = (legTime[i]+incrementTime)%gaitPeriod;
    }
}

bool bellyRoll(){
  sensors_event_t event;
  bno.getEvent(&event);
  float orientation = (float)event.orientation.z;
  Serial.print("orientation: );
  Serial.println(orientation);
  if (rollStep == 0)
  {
    if (orientation < 20 && orientation > -20){
        for (int i = 0; i < 4;i+=2)
        {
          Legs[i].m2.moveToDegree(-35);
          Legs[i].m3.moveToDegree(30);
        }
        for (int i = 0; i <4; i +=2)
        {
          Legs[i].m2.setSpeed(1);
          Legs[i].m3.setSpeed(1);
          Legs[i].m2.setDestinationDegree(-100);
          Legs[i].m3.setDestinationDegree(0); 
        }
        rollStep = 1;
      }
  }
  else
  {
    for (int i = 0; i < 4;i+=2)
      {
        if (Legs[i].stepAll())
        {
          Legs[i].m3.setSpeed(1);
          Legs[i].m3.setDestinationDegree(-100);
        }
      }
      if (orientation < -95)
      {
        rightSideUp = false;
        rollStep = 0;
        for (int i = 0; i < 4;i+=2)
        {
          Legs[i].m2.moveToDegree(-stowPos[1]);
          Legs[i].m3.moveToDegree(stowPos[2]);
        }
        return true;
      }
  }
  /*
  switch(rollStep) {
    case 0 :
      if (orientation < 20){
        for (int i = 0; i < 4;i+=2)
        {
          Legs[i].m2.moveToDegree(-35);
          Legs[i].m3.moveToDegree(30);
        }
        for (int i = 0; i <4; i +=2)
        {
          Legs[i].m2.setSpeed(0.25);
          Legs[i].m3.setSpeed(0.5);
          Legs[i].m2.setDestinationDegree(-100);
          Legs[i].m3.setDestinationDegree(0); 
        }
        rollStep = 1;
      }
      break;
    case 1:
      for (int i = 0; i < 4;i+=2)
      {
        if (Legs[i].stepAll())
        {
          Legs[i].m3.setSpeed(0.5);
          Legs[i].m3.setDestinationDegree(-100);
        }
      }
      if (orientation < -95)
      {
        rightSideUp = false;
        rollStep = 0;
        for (int i = 0; i < 4;i+=2)
        {
          Legs[i].m2.moveToDegree(-stowPos[1]);
          Legs[i].m3.moveToDegree(stowPos[2]);
        }
        return true;
      }
      break;
  }*/
  return false;
}

bool backRoll(){
  sensors_event_t event;
  bno.getEvent(&event);
  float orientation = (float)event.orientation.z;
  if (rollStep == 0)
  {
    for (int i = 1; i < 4;i+=2)
        {
          Legs[i].m2.moveToDegree(35);
          Legs[i].m3.moveToDegree(-30);
        }
        for (int i = 1; i <4; i +=2)
        {
          Legs[i].m2.setSpeed(0.35);
          Legs[i].m3.setSpeed(0.5);
          Legs[i].m2.setDestinationDegree(100);
          Legs[i].m3.setDestinationDegree(0); 
        }
        rollStep = 1;
  }
  else
  {
    for (int i = 1; i < 4;i+=2)
      {
        if (Legs[i].stepAll())
        {
          Legs[i].m3.setSpeed(0.5);
          Legs[i].m3.setDestinationDegree(100);
        }
      }
        if ((orientation < 85) && (orientation > -85))
        {
          rightSideUp = true;
          rollStep = 0;
          for (int i = 1; i < 4;i+=2)
          {
            Legs[i].m2.moveToDegree(stowPos[1]);
            Legs[i].m3.moveToDegree(-stowPos[2]);
          }
          return true;
        }
  }/*
  switch(rollStep) {
    case 0 :
      if (orientation < -160 || orientation > 0){
        for (int i = 1; i < 4;i+=2)
        {
          Legs[i].m2.moveToDegree(35);
          Legs[i].m3.moveToDegree(-30);
        }
        for (int i = 1; i <4; i +=2)
        {
          Legs[i].m2.setSpeed(0.25);
          Legs[i].m3.setSpeed(0.5);
          Legs[i].m2.setDestinationDegree(100);
          Legs[i].m3.setDestinationDegree(0); 
        }
        rollStep = 1;
        break;
      }
    case 1:
      for (int i = 1; i < 4;i+=2)
      {
        if (Legs[i].stepAll())
        {
          Legs[i].m3.setSpeed(0.5);
          Legs[i].m3.setDestinationDegree(100);
        }
      }
        if ((orientation < 85) && (orientation > -85))
        {
          rightSideUp = true;
          rollStep = 0;
          for (int i = 1; i < 4;i+=2)
          {
            Legs[i].m2.moveToDegree(stowPos[1]);
            Legs[i].m3.moveToDegree(-stowPos[2]);
          }
          return true;
        }
        break;
  }*/
  return false;
}

// Interpolation function
float interpolate(float val,float minIN,float maxIN,float minOUT, float maxOUT){
  return (val-minIN)*(maxOUT-minOUT)/(maxIN-minIN)+minOUT;
}

bool orientationCheck(){
  sensors_event_t event;
  bno.getEvent(&event);
  float orientation = (float)event.orientation.z;
  if (abs(orientation) < 90)
    return true;
  else
    return false;
}



// Function to move motors slowly. the inputs are the number of motors to be moved, an array containing the motors to be moved, the desired position (in degrees), and the moving speed (degrees/millisecond)
void moveSlowly(int numMotors,scottoMotorInterface* motors2move,float desPosition,float movespeed)
{
  float tol = 0.01;
  bool reached = false;
  float lastCommand[numMotors];
  for (int i=0;i<numMotors;i++)
  {
    delay(5);
    lastCommand[i] = motors2move[i].readDegree();
  }
  uint32_t startTime = millis();
  uint32_t moveTime;
  delay(1);
  while (!reached)
  {
    reached = true;
    moveTime = millis()-startTime;
    startTime = millis();
    for (int i =0;i<numMotors;i++)
    {
      float commandChange = constrain(desPosition - lastCommand[i],-movespeed*(float)moveTime,movespeed*(float)moveTime);
      lastCommand[i] = lastCommand[i]+commandChange;
      motors2move[i].moveToDegree(lastCommand[i]);
      if (abs(commandChange) > tol)
        reached = false;
    }
    delay(1);
  } 
}

void moveSlowlyMultiGoal(int numMotors,scottoMotorInterface* motors2move,float* desPosition,float movespeed)
{
  float tol = 0.01;
  bool reached = false;
  float lastCommand[numMotors];
  for (int i=0;i<numMotors;i++)
  {
    delay(5);
    lastCommand[i] = motors2move[i].readDegree();
  }
  uint32_t startTime = millis();
  uint32_t moveTime;
  delay(1);
  while (!reached)
  {
    reached = true;
    moveTime = millis()-startTime;
    startTime = millis();
    for (int i =0;i<numMotors;i++)
    {
      float commandChange = constrain(desPosition[i] - lastCommand[i],-movespeed*(float)moveTime,movespeed*(float)moveTime);
      lastCommand[i] = lastCommand[i]+commandChange;
      motors2move[i].moveToDegree(lastCommand[i]);
      if (abs(commandChange) > tol)
        reached = false;
    }
    delay(1);
  } 
}

bool standTest()
{
  // start by moving motor 1 slowly to the middle of the walking gait
  if (standStep == 0)
  {
    for (int i = 0;i< 4;i+=2)
    {
        Legs[i].m1.moveToDegree(frontM1Goal);
        Legs[i].m1.moveToDegree(frontM1Goal);
    }
    for (int i = 1;i<4; i+=2)
    {
        Legs[i].m1.moveToDegree(backM1Goal);
        Legs[i].m1.moveToDegree(backM1Goal);
    }
    standTime = millis();
    standStep = 1;
  }
  else if (millis()-standTime > 1000 && standStep == 1)
  {
    for (int i = 0;i<4;i++)
    {
      Legs[i].m2.moveToDegree(m2Goal1);
    }
    standStep = 2;
  }
  else if (millis()-standTime > 1500 && standStep == 2) {
    for (int i=0;i<4;i++)
    {
      Legs[i].m3.moveToDegree(m3Goal);
    }
    standStep = 3;
  }
  else if (millis()-standTime > 2000 && standStep == 3) {
    for (int i=0;i<4;i++)
    {
      Legs[i].m2.moveToDegree(m2Goal2);
    }
    standStep = 0;
    return true;
  }
  return false;
}


// Function to stand
/*
void stand(float frontM1Goal, float backM1Goal, float m2Goal1, float m2Goal2, float m3Goal)
{
  float movespeed = 0.2; //degrees per millisecond
  
  // Start by moving motor 1 slowly to the middle of the walking gait
  float frontM1Goal = 0;
  float backM1Goal = 0;
  float m2Goal1 = 0;
  float m2Goal2 = 0;
  float m3Goal = 0;
  for (int i = 0; i < (sizeof(motor_1Front)/sizeof(motor_1Front[0]));i++)
  {
    frontM1Goal += motor_1Front[i];
    backM1Goal += motor_1Back[i];
    m2Goal2 = min(m2Goal2,motor_2Front[i]);
    m2Goal1 = max(m2Goal1,motor_2Front[i]);
    m3Goal = min(m3Goal,motor_3Front[i]);
    m2Goal1 = max(m2Goal1,motor_2Back[i]);
    m2Goal2 = min(m2Goal2,motor_2Back[i]);
    m3Goal = min(m3Goal,motor_3Back[i]);
  }
  frontM1Goal = frontM1Goal/(float)(sizeof(motor_1Front)/sizeof(motor_1Front[0]));
  backM1Goal = backM1Goal/(float)(sizeof(motor_1Front)/sizeof(motor_1Front[0]));
  scottoMotorInterface motors2move[] = {Legs[0].m1,Legs[2].m1};
  
  moveSlowly(2,motors2move,frontM1Goal,movespeed);
  motors2move[0] = Legs[1].m1;
  motors2move[1] = Legs[3].m1;
  moveSlowly(2,motors2move,backM1Goal,movespeed);
  scottoMotorInterface motors2move2[4];
  for (int i = 0; i <4;i++)
    motors2move2[i] = Legs[i].m2;
  moveSlowly(4,motors2move2,m2Goal1,movespeed);
  for (int i = 0; i <4;i++)
    motors2move2[i] = Legs[i].m3;
  moveSlowly(4,motors2move2,m3Goal,movespeed);
  for (int i = 0; i <4;i++)
    motors2move2[i] = Legs[i].m2;
  moveSlowly(4,motors2move2,m2Goal2,movespeed);

} */

bool sitTest()
{
  float neutralPos = 0;  
  float m2Goal1 = 45;
  
  if (sitCommanded && sitStep == 0) {
    for (int i = 0;i<4;i++)
    {
      // Legs[i].m2.setSpeed(0.02);
      // Legs[i].m2.setDestinationDegree(m2Goal1);
      Legs[i].m2.moveToDegree(m2Goal1);
    }
    sitTime = millis();
    sitStep = 1;
  }

  // go to neutral position for all motors
  if (millis() - sitTime > 1000 && sitStep == 1) {
    for (int i=0;i<4;i++) {
      Legs[i].m1.moveToDegree(neutralPos);
      Legs[i].m2.moveToDegree(neutralPos);
      Legs[i].m3.moveToDegree(neutralPos);
    }
    sitStep = 0;
    sitCommanded = false; // done sitting
    return true;
  }
  return false;
}

bool stowTest()
{
  if (stowStep == 0) {
    // set motor 3 position (feet) at same time
    for (int i=0;i<4;i+=2) {
      Legs[i].m3.moveToDegree(stowPos[2]);
      Legs[i].m2.moveToDegree(-stowPos[1]);
    }
    for (int i=1;i<4;i+=2) {
      Legs[i].m3.moveToDegree(-stowPos[2]);
      Legs[i].m2.moveToDegree(stowPos[1]);
    }
    stowTime = millis();
    stowStep = 1;
  }
  else if (millis() - stowTime > 1000 && stowStep == 1) {
    // now, move motor 1 to stow
    for (int i=0;i<4;i+=2){
      Legs[i].m1.moveToDegree(-stowPos[0]);
    }
    for (int i=1;i<4;i+=2){
      Legs[i].m1.moveToDegree(stowPos[0]);
    }
    stowStep = 0;
    return true;
  }
    
    /*
    for (int i=0;i<4;i+=2)
    {
      Legs[i].m1.setSpeed(0.05);
      Legs[i].m1.setDestinationDegree(-stowPos[0]);
    }
    for (int i=1;i<4;i+=2)
    {
      Legs[i].m1.setSpeed(0.05);
      Legs[i].m1.setDestinationDegree(stowPos[0]);
    }
    for (int i=0;i<4;i++) {
        if (Legs[i].stepAll()) {
        sitCommanded = false;
        return true;
        }
     }
  }  */  
  return false; // default case
}

/*
// Function to lower legs to sit on body, then stow legs to get ready for roll
void sit()
{
  float movespeed = 0.5; //degrees per millisecond
  
  // Lift off ground with motor 2
  float m2Goal1 = 45;
  scottoMotorInterface allLegMotors[4];
  for (int i = 0; i <4;i++)
    allLegMotors[i] = Legs[i].m2;
  moveSlowly(4,allLegMotors,m2Goal1,movespeed);
  
  //Set motor 3 position
  movespeed = 0.5;
  scottoMotorInterface twoLegMotors[2];
  twoLegMotors[0] = Legs[0].m3;
  twoLegMotors[1] = Legs[2].m3;
  moveSlowly(2,twoLegMotors,stowPos[2],movespeed);
  twoLegMotors[0] = Legs[1].m3;
  twoLegMotors[1] = Legs[3].m3;
  moveSlowly(2,twoLegMotors,-stowPos[2],movespeed);
  
  //Set motor 2 position
  twoLegMotors[0] = Legs[0].m2;
  twoLegMotors[1] = Legs[2].m2;
  moveSlowly(2,twoLegMotors,-stowPos[1],movespeed);
  twoLegMotors[0] = Legs[1].m2;
  twoLegMotors[1] = Legs[3].m2;
  moveSlowly(2,twoLegMotors,stowPos[1],movespeed);
  
  movespeed = 0.05;
  //Set motor 1 position
  twoLegMotors[0] = Legs[0].m1;
  twoLegMotors[1] = Legs[2].m1;
  moveSlowly(2,twoLegMotors,-stowPos[0],movespeed);
  twoLegMotors[0] = Legs[1].m1;
  twoLegMotors[1] = Legs[3].m1;
  moveSlowly(2,twoLegMotors,stowPos[0],movespeed);
} */
