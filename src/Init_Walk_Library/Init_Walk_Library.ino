// Author: ScottoBots

#define STOW_M1 85
#define STOW_M2 70
#define STOW_M3 80

// import ax12 library to send dynamixel commands and libraries for interfacing with motors
#include <ax12.h>
#include "scottoLeg.h"
#include "scottoMotorInterface.h"

//Time of the gait
uint16_t gaitPeriod = 2000;

//Trajectory of the three motors on each leg

float motor_1Front[] = {42,42,41,41,40,39,38,37,35,34,33,31,30,28,27,24,23,21,19,17,15,13,11,9,6,3,0,-1,-2,-2,-1.2497,1.2221,5.2152,10.4061,16.3742,22.636,28.6843,34.0291,38.2373,40.968};
float motor_2Front[] = {-27,-27,-27,-27,-27,-28,-27,-27,-27,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-20.7816,-15.0265,-10.2009,-6.6958,-4.7951,-4.6528,-6.2804,-9.5461,-14.1853,-19.8221};
float motor_3Front[] = {-41,-41,-43,-44,-46,-47,-48,-50,-52,-53,-55,-57,-58,-60,-61,-62,-63,-64,-64,-65,-65,-66,-66,-66,-67,-67,-67,-67,-67,-67,-67,-64.1111,-61.2222,-58.3333,-55.4444,-52.5556,-49.6667,-46.7778,-43.8889,-41};
float motor_1Back[] = {-40.968,-38.2373,-34.0291,-28.6843,-22.636,-16.3742,-10.4061,-5.2152,-1.2221,1.2497,2,2,1,0,-3,-6,-9,-11,-13,-15,-17,-19,-21,-23,-24,-27,-28,-30,-31,-33,-34,-35,-37,-38,-39,-40,-41,-41,-42,-42};
float motor_2Back[] = {-19.8221,-14.1853,-9.5461,-6.2804,-4.6528,-4.7951,-6.6958,-10.2009,-15.0265,-20.7816,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-27,-27,-27,-28,-27,-27,-27,-27,-27};
float motor_3Back[] = {-41,-43.8889,-46.7778,-49.6667,-52.5556,-55.4444,-58.3333,-61.2222,-64.1111,-67,-67,-67,-67,-67,-67,-67,-66,-66,-66,-65,-65,-64,-64,-63,-62,-61,-60,-58,-57,-55,-53,-52,-50,-48,-47,-46,-44,-43,-41,-41};

float timeBetweenGaitPos;

// trajectory generated from simulation
/*float motor1Sim[] = {-37.6978,-36.1457,-34.1475,-32.0502,-29.8521,-27.5528,-25.1531,-22.6552,-20.0631,-17.3824,-14.6209,-11.7882,-8.8959,-5.9571,-2.9866,-5.9798e-06,2.9866,5.9571,8.8959,11.7882,14.6209,17.3824,20.0631,22.6552,25.1531,27.5528,29.8521,32.0501,34.1475,36.1457,37.6978,38.047,37.5329,30.5523,15.7346,-1.793,-19.001,-32.7419,-37.8865,-38.0435,-37.4821};
float motor2Sim[] = {31.9334,31.3953,30.8887,30.5233,30.2725,30.1142,30.0292,30.0004,30.0128,30.0527,30.108,30.1684,30.2248,30.2703,30.2997,30.3099,30.2997,30.2703,30.2248,30.1684,30.108,30.0527,30.0128,30.0004,30.0292,30.1142,30.2725,30.5233,30.8887,31.3953,30.5006,24.7958,18.3159,14.6013,15.0298,15.4629,14.8634,14.814,18.926,25.4759,30.8248};
float motor3Sim[] = {46.2918,48.5928,51.1142,53.3364,55.2944,57.0151,58.5197,59.8258,60.9475,61.8969,62.6838,63.3166,63.8019,64.1448,64.3491,64.417,64.3491,64.1448,63.8019,63.3166,62.6838,61.8969,60.9475,59.8258,58.5197,57.0151,55.2944,53.3364,51.1142,48.5928,48.2758,55.5716,64.3347,74.8134,81.6453,83.4529,80.7167,72.9028,63.2171,54.6821,48.0523};
*/

float motor1Sim[] = {-37.6978,-36.53,-34.9586,-33.3206,-31.6186,-29.8521,-28.0207,-26.1249,-24.1656,-22.1442,-20.0631,-17.9253,-15.7346,-13.4957,-11.2141,-8.8959,-6.5479,-4.1777,-1.793,0.59784,2.9866,5.3651,7.7251,10.0591,12.3599,14.6209,16.8363,19.001,21.1109,23.1625,25.1531,27.0809,28.9445,30.7434,32.4776,34.1475,35.7539,37.211,37.9618,38.047,37.5329,32.742,22.1442,8.8959,-5.3651,-19.001,-30.5523,-36.8902,-38.047,-38.0203,-37.4821};
float motor2Sim[] = {31.9334,31.5151,31.0727,30.7272,30.4647,30.2725,30.1394,30.0555,30.0118,30.0001,30.0128,30.0431,30.0847,30.1321,30.1802,30.2248,30.2624,30.2902,30.3062,30.3095,30.2997,30.2776,30.2447,30.2032,30.1564,30.108,30.0629,30.0261,30.0038,30.0024,30.0292,30.0919,30.1992,30.3606,30.5864,30.8887,31.2812,31.3974,28.6801,23.4395,18.3159,14.9117,14.7052,15.3183,15.4132,14.8634,14.5669,16.7811,21.4503,26.814,30.8248};
float motor3Sim[] = {46.2918,48.054,50.1439,52.0368,53.7482,55.2944,56.6888,57.9426,59.0651,60.0645,60.9475,61.7203,62.388,62.955,63.4253,63.8019,64.0874,64.2839,64.3926,64.4143,64.3491,64.1967,63.9559,63.6251,63.202,62.6838,62.067,61.3473,60.5201,59.5797,58.5197,57.3327,56.0099,54.5411,52.9141,51.1142,49.1231,47.5377,50.4858,57.3366,64.3347,72.7868,79.5811,82.9177,83.2752,80.7167,74.8541,66.8538,59.8849,52.9329,48.0523};

scottoLeg Legs[4];

// leg / motor properties
int leg0[] = {1,2,3}; //front left
int leg1[] = {7,8,9}; //back left
int leg2[] = {12,11,10}; //front right
int leg3[] = {4,5,6}; //back right

//time used to track position in trajectory for each leg.
uint16_t legTime[4];

// min/maxes of all motors
int motorMins[] = {850,859,865,159,864,156,860,157,861,863,861,861}; // 156 -> 861 for motor 12 (backwards)
int motorMaxs[] = {159,159,159,869,158,860,156,863,160,157,156,156}; // NOTE: swapped min and max for 4, 5 

float stowPos[] = {STOW_M1,STOW_M2,STOW_M3};
float kickInitPos[] = {85,30,0};

void setup(){
   //open serial port
   Serial.begin(9600);
   delay(500);
   
   //Initialize all motors on all legs
   Legs[0] = scottoLeg(leg0[0],leg0[1],leg0[2],motorMins[leg0[0]-1],motorMaxs[leg0[0]-1],motorMins[leg0[1]-1],motorMaxs[leg0[1]-1],motorMins[leg0[2]-1],motorMaxs[leg0[2]-1]);
   Legs[1] = scottoLeg(leg1[0],leg1[1],leg1[2],motorMins[leg1[0]-1],motorMaxs[leg1[0]-1],motorMins[leg1[1]-1],motorMaxs[leg1[1]-1],motorMins[leg1[2]-1],motorMaxs[leg1[2]-1]);
   Legs[2] = scottoLeg(leg2[0],leg2[1],leg2[2],motorMins[leg2[0]-1],motorMaxs[leg2[0]-1],motorMins[leg2[1]-1],motorMaxs[leg2[1]-1],motorMins[leg2[2]-1],motorMaxs[leg2[2]-1]);
   Legs[3] = scottoLeg(leg3[0],leg3[1],leg3[2],motorMins[leg3[0]-1],motorMaxs[leg3[0]-1],motorMins[leg3[1]-1],motorMaxs[leg3[1]-1],motorMins[leg3[2]-1],motorMaxs[leg3[2]-1]);
   
   // Stagger the time that the legs start at in the gait cycle
   legTime[0] = 0;
   legTime[1] = gaitPeriod/2;
   legTime[2] = gaitPeriod/4;
   legTime[3] = gaitPeriod*3/4;
   
   // Compute the time between each position in the trajectory
   timeBetweenGaitPos = (float)gaitPeriod/((float)(sizeof(motor_1Front)/sizeof(motor_1Front[0])-1));
   Serial.println("\nStarting in 2 seconds");
   delay(2000);
   Serial.println("starting...");
   stand();
   delay(1000);
   // sit();
   delay(1000);
   // frontLegRoll();
   // stow();
 }

void loop(){
  // relax all legs and read position (need delays)
  /* Legs[0].relaxAll();
  Serial.print(Legs[0].m1.readDegree());
  delay(10);
  Serial.print(",");
  Serial.print(Legs[0].m2.readDegree());
  delay(10);
  Serial.print(",");
  Serial.println(Legs[0].m3.readDegree());
  delay(10); */ 
  
  uint32_t startTime = millis();
  
  // move each leg
//  for (int i = 0; i < 4; i++)
//    moveFrontLeg(Legs[i],legTime[i]);
    
//  for (int i = 1; i < 4; i+=2)
//    moveBackLeg(Legs[i],legTime[i]);

  for (int i=0; i<4; i++)
    moveLegSim(Legs[i], legTime[i]);
 
 // Increment each leg's time in the gait cycle 
  int incrementTime = millis() - startTime;
  for (int i = 0; i < 4; i ++)
    legTime[i] = (legTime[i]+incrementTime) % gaitPeriod;
}

// Function to move a front leg to the position given a time in a gait cycle
void moveFrontLeg(scottoLeg leg2move,uint16_t gaitTime)
{
  int gaitNum = floor((float)gaitTime/timeBetweenGaitPos); // find the 
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

// move leg based on simulation outputs
void moveLegSim(scottoLeg legCommand, uint16_t gaitTime)
{
  int gaitIdx = floor((float)gaitTime / timeBetweenGaitPos); // find index in current gait cycle
  float command1 = interpolate(gaitTime, timeBetweenGaitPos * (float)gaitIdx, timeBetweenGaitPos * (float)(gaitIdx+1), motor1Sim[gaitIdx], motor1Sim[gaitIdx+1]);
  float command2 = interpolate(gaitTime, timeBetweenGaitPos * (float)gaitIdx, timeBetweenGaitPos * (float)(gaitIdx+1), motor2Sim[gaitIdx], motor2Sim[gaitIdx+1]);
  float command3 = interpolate(gaitTime, timeBetweenGaitPos * (float)gaitIdx, timeBetweenGaitPos * (float)(gaitIdx+1), motor3Sim[gaitIdx], motor3Sim[gaitIdx+1]);
  legCommand.moveAllDegree(command1, command2, command3);
}

// Function to stand
void stand()
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
  movespeed = 10;
  for (int i = 0; i <4;i++)
    motors2move2[i] = Legs[i].m2;
  moveSlowly(4,motors2move2,m2Goal2,movespeed);

}

// Function to lower legs to sit on body
void sit()
{
  float movespeed = 0.05; //degrees per millisecond
  
  // Lift off ground with motor 2
  float m2Goal1 = 45;
  scottoMotorInterface allLegMotors[4];
  for (int i = 0; i<4;i++)
    allLegMotors[i] = Legs[i].m2;
  moveSlowly(4,allLegMotors,m2Goal1,movespeed);
}

// Stow legs to get ready for roll
void stow()
{ 
  //Set motor 3 position
  float movespeed = 0.2;
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
  
  //Set motor 1 position
  twoLegMotors[0] = Legs[0].m1;
  twoLegMotors[1] = Legs[2].m1;
  moveSlowly(2,twoLegMotors,-stowPos[0],movespeed);
  twoLegMotors[0] = Legs[1].m1;
  twoLegMotors[1] = Legs[3].m1;
  moveSlowly(2,twoLegMotors,stowPos[0],movespeed);
}

void walk()
{
  
}

// Function to use front legs to roll over
void frontLegRoll()
{
  float movespeed = 0.1; //degrees per millisecond
  float m2Goal = -90;
  float m3Goal = -90;
  
  scottoMotorInterface twoLegMotors[2];
  //Set motor 2 position
  twoLegMotors[0] = Legs[0].m2;
  twoLegMotors[1] = Legs[2].m2;
  moveSlowly(2,twoLegMotors,m2Goal,movespeed);
  
  //Set motor 3 position
  twoLegMotors[0] = Legs[0].m3;
  twoLegMotors[1] = Legs[2].m3;
  moveSlowly(2,twoLegMotors,m3Goal,movespeed);
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

// Interpolation function
float interpolate(float val,float minIN,float maxIN,float minOUT, float maxOUT){
  return (val-minIN)*(maxOUT-minOUT)/(maxIN-minIN)+minOUT;
}
