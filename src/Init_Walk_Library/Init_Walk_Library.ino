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
uint16_t lastWalkTime = 0;

bool standCommanded = true;
bool sitCommanded = true;
bool walkCommanded = true;

//Time of the gait
uint16_t gaitPeriod = 1000;
//Trajectory of the three motors on each leg
/*
float motor_1Front[] = {42,42,41,41,40,39,38,37,35,34,33,31,30,28,27,24,23,21,19,17,15,13,11,9,6,3,0,-1,-2,-2,-1.2497,1.2221,5.2152,10.4061,16.3742,22.636,28.6843,34.0291,38.2373,40.968};
float motor_2Front[] = {-27,-27,-27,-27,-27,-28,-27,-27,-27,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-20.7816,-15.0265,-10.2009,-6.6958,-4.7951,-4.6528,-6.2804,-9.5461,-14.1853,-19.8221};
float motor_3Front[] = {-41,-41,-43,-44,-46,-47,-48,-50,-52,-53,-55,-57,-58,-60,-61,-62,-63,-64,-64,-65,-65,-66,-66,-66,-67,-67,-67,-67,-67,-67,-67,-64.1111,-61.2222,-58.3333,-55.4444,-52.5556,-49.6667,-46.7778,-43.8889,-41};
float motor_1Back[] = {-40.968,-38.2373,-34.0291,-28.6843,-22.636,-16.3742,-10.4061,-5.2152,-1.2221,1.2497,2,2,1,0,-3,-6,-9,-11,-13,-15,-17,-19,-21,-23,-24,-27,-28,-30,-31,-33,-34,-35,-37,-38,-39,-40,-41,-41,-42,-42};
float motor_2Back[] = {-19.8221,-14.1853,-9.5461,-6.2804,-4.6528,-4.7951,-6.6958,-10.2009,-15.0265,-20.7816,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-27,-27,-27,-28,-27,-27,-27,-27,-27};
float motor_3Back[] = {-41,-43.8889,-46.7778,-49.6667,-52.5556,-55.4444,-58.3333,-61.2222,-64.1111,-67,-67,-67,-67,-67,-67,-67,-66,-66,-66,-65,-65,-64,-64,-63,-62,-61,-60,-58,-57,-55,-53,-52,-50,-48,-47,-46,-44,-43,-41,-41};
*/


float timeBetweenGaitPos;

// trajectory generated from simulation
/*float motor_1Front[] = {38.6757,37.1323,35.5234,33.8472,32.1027,30.2888,28.4053,26.4524,24.431,22.3427,20.1899,17.9759,15.705,13.3824,11.0141,8.6072,6.1692,3.7086,1.2343,-1.2447,-3.719,-6.1795,-8.6173,-11.0242,-12.8043,-12.8043,-12.8043,-10.4258,-0.62505,9.2121,18.5349,26.9471,34.2726,40.5154,40.5154,40.5154,40.5154,39.0515};
float motor_2Front[] = {-41.6261,-40.8398,-40.2284,-39.7561,-39.3967,-39.1298,-38.9387,-38.8097,-38.7307,-38.6914,-38.6824,-38.6956,-38.7235,-38.7598,-38.7988,-38.8358,-38.8669,-38.8893,-38.9009,-38.9009,-38.8892,-38.8668,-38.8356,-38.7986,-37.0201,-30.4694,-24.3897,-19.4093,-19.6457,-19.4601,-18.95,-18.4036,-18.2254,-18.8857,-25.4637,-32.7784,-41.624,-41.8556};
float motor_3Front[] = {-34.6481,-37.3827,-39.7843,-41.9095,-43.7985,-45.4807,-46.9791,-48.3115,-49.4926,-50.5344,-51.4468,-52.2385,-52.9164,-53.4868,-53.9545,-54.3238,-54.5978,-54.7791,-54.8693,-54.8691,-54.7786,-54.5969,-54.3224,-53.9527,-56.1426,-65.3466,-73.477,-80.3293,-81.0083,-80.4824,-78.7016,-75.5115,-70.6671,-63.8026,-55.401,-45.4553,-32.6083,-33.9033};
float motor_1Back[] = {9.8251,7.4018,4.9515,2.483,0.0052149,-2.4726,-4.9412,-7.3915,-9.815,-12.2036,-14.5498,-16.8473,-19.0903,-21.2741,-23.3951,-25.4502,-27.4375,-29.3558,-31.2044,-32.9836,-34.6938,-36.3362,-37.9121,-39.4233,-40.5154,-40.5154,-40.5154,-39.0515,-32.5453,-24.9427,-16.2778,-6.7812,3.1013,12.8042,12.8043,12.8043,12.8043,10.4258};
float motor_2Back[] = {-38.8176,-38.8521,-38.8792,-38.8965,-38.9024,-38.8965,-38.8793,-38.8523,-38.8178,-38.7793,-38.741,-38.7081,-38.6867,-38.6837,-38.7067,-38.7646,-38.8672,-39.0257,-39.2528,-39.5637,-39.9767,-40.5148,-41.2084,-42.1001,-40.3747,-31.8053,-24.6101,-18.6135,-18.2108,-18.5228,-19.0948,-19.5445,-19.625,-19.2943,-25.1317,-31.257,-37.8877,-38.8082};
float motor_3Back[] = {-54.1497,-54.4714,-54.6992,-54.8352,-54.8805,-54.8355,-54.7,-54.4725,-54.1512,-53.7332,-53.2147,-52.5913,-51.8573,-51.0062,-50.0302,-48.9202,-47.665,-46.2517,-44.6639,-42.8815,-40.8787,-38.6211,-36.0615,-33.131,-34.476,-46.8144,-56.5207,-65.7357,-72.0504,-76.453,-79.2713,-80.7277,-80.9521,-79.9662,-72.5068,-64.2638,-54.8907,-54.0543};
*/

// trajectory generated from simulation (2: 4/28/19)
float motor_1Front[] = {45,44.1275,43.2285,42.3022,41.3478,40.3645,39.3518,38.3087,37.2348,36.1294,34.992,33.8221,32.6192,31.3832,30.1137,28.8108,27.4744,26.1049,24.7024,23.2677,21.8014,20.3045,18.778,17.2234,15.6423,14.0362,12.4074,10.758,9.0903,7.4069,5.7106,4.0042,2.2906,0.57294,3.7632e-15,-1.1001e-15,2.3148e-15,2.5577e-15,4.5739,11.3099,17.7447,23.7495,29.2488,34.2157,38.6598,42.614,45,45,45,45};
float motor_2Front[] = {-42.8576,-41.7132,-40.7706,-39.9832,-39.3201,-38.7599,-38.2864,-37.8875,-37.5531,-37.2752,-37.0471,-36.8627,-36.7171,-36.6056,-36.5242,-36.4691,-36.4367,-36.424,-36.428,-36.4457,-36.4746,-36.5121,-36.5561,-36.6042,-36.6545,-36.7052,-36.7544,-36.8008,-36.8428,-36.8795,-36.9097,-36.9326,-36.9478,-36.9548,-33.0796,-27.5893,-22.349,-17.2534,-14.6799,-14.4041,-13.9537,-13.4286,-12.954,-12.6667,-12.7071,-13.2191,-15.7689,-21.6053,-27.9583,-35.2433};
float motor_3Front[] = {-26.7194,-29.7084,-32.3435,-34.7043,-36.8429,-38.7951,-40.5875,-42.2398,-43.7675,-45.1831,-46.4965,-47.7163,-48.8494,-49.9017,-50.8786,-51.7845,-52.6234,-53.3988,-54.1139,-54.7715,-55.3741,-55.9242,-56.4238,-56.8747,-57.2788,-57.6375,-57.9522,-58.2241,-58.4543,-58.6437,-58.7931,-58.903,-58.9739,-59.0061,-64.5124,-72.0417,-78.9174,-85.2956,-88.2135,-87.5884,-86.3998,-84.6019,-82.1342,-78.924,-74.8824,-69.8923,-63.7349,-56.5127,-48.1741,-38.0123};
float motor_1Back[] = {0,-1.7184,-3.4336,-5.1428,-6.8428,-8.5308,-10.204,-11.8598,-13.4957,-15.1096,-16.6992,-18.2629,-19.7989,-21.3058,-22.7824,-24.2277,-25.641,-27.0216,-28.369,-29.6831,-30.9638,-32.2109,-33.4248,-34.6057,-35.7539,-36.8699,-37.9542,-39.0075,-40.0303,-41.0233,-41.9872,-42.9228,-43.8309,-44.7121,-45,-45,-45,-45,-42.6141,-38.6598,-34.2157,-29.2488,-23.7495,-17.7447,-11.31,-4.5739,-2.292e-10,7.2233e-15,-7.5562e-16,5.89e-15};
float motor_2Back[] = {-36.9552,-36.9511,-36.9386,-36.9182,-36.8903,-36.8557,-36.8153,-36.7703,-36.7218,-36.6715,-36.6208,-36.5718,-36.5262,-36.4862,-36.4542,-36.4325,-36.4236,-36.4305,-36.4559,-36.5031,-36.5753,-36.6764,-36.8101,-36.981,-37.1939,-37.4545,-37.7693,-38.1457,-38.5931,-39.1228,-39.7495,-40.4927,-41.3794,-42.4499,-36.6079,-29.0908,-22.6209,-16.7157,-13.2191,-12.7071,-12.6667,-12.954,-13.4286,-13.9537,-14.4041,-14.6799,-16.4125,-21.492,-26.7018,-32.1413};
float motor_3Back[] = {-59.0083,-58.9889,-58.931,-58.8341,-58.698,-58.522,-58.3054,-58.0475,-57.7472,-57.4033,-57.0145,-56.5794,-56.0963,-55.5632,-54.9783,-54.3393,-53.6437,-52.8887,-52.0714,-51.1882,-50.2355,-49.2088,-48.1033,-46.9132,-45.6318,-44.2513,-42.7622,-41.153,-39.4093,-37.5129,-35.4399,-33.158,-30.6214,-27.7617,-36.0386,-46.6361,-55.2128,-62.5922,-69.8923,-74.8823,-78.9239,-82.1342,-84.6018,-86.3998,-87.5884,-88.2135,-86.318,-80.0118,-73.228,-65.822};

// angles for standing position
float frontM1Goal = 0;
float backM1Goal = 0;
float m2Goal1 = 0;
float m2Goal2 = 0;
float m3Goal1 = 0;
float m3Goal2 = 0;

void setup(){
   //open serial port
   Serial.begin(9600);
   SD.begin(4);
   myFile = SD.open("test.txt", FILE_WRITE);
   // Set up IMU
   Serial.println("\n Starting IMU \n");
   //while(!bno.begin())
   //  Serial.println(".");
   Serial.println("here");
   //bno.setExtCrystalUse(true);
   //sensors_event_t event;
   //bno.getEvent(&event);
   delay(100);
   //rightSideUp = orientationCheck;
   
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
    //m2Goal2 = min(m2Goal2,motor_2Front[i]);
    //m2Goal1 = max(m2Goal1,motor_2Front[i]);
    //m3Goal = min(m3Goal,motor_3Front[i]);
    //m2Goal1 = max(m2Goal1,motor_2Back[i]);
    //m2Goal2 = min(m2Goal2,motor_2Back[i]);
    //m3Goal = min(m3Goal,motor_3Back[i]);
  }
  m2Goal1 = 30;
  m3Goal1 = -90;
  m2Goal2 = -45;
  m3Goal2 = -30;
  frontM1Goal = frontM1Goal/(float)(sizeof(motor_1Front)/sizeof(motor_1Front[0]));
  backM1Goal = backM1Goal/(float)(sizeof(motor_1Front)/sizeof(motor_1Front[0]));
   
   
   Serial.println("\nStarting in 2 seconds");
   delay(2000);
  while (!standTest());
  //delay(3000);
  //routineTime = millis(); // start of routine
  
  for (int i = 0; i < 4; i ++)
  {
    Legs[i].m1.moveToDegree(0);
    Legs[i].m2.moveToDegree(-90);
    Legs[i].m3.moveToDegree(0);
  }
  while(1);
 }

void loop(){  
  if (millis() - routineTime > 10000){
    myFile.close();
    while(1);
  }
  else if (millis() - routineTime > 14000) {
    writetoCard();
      if (bellyRoll()) {
        rollCount++;
      }
    if (rollCount > 4) {
      //standTest(); // stop rolling
      myFile.close();
      while(1);
    }
  }
  else if (millis() - routineTime > 12000) {
    stowTest();
  }
  else if (millis() - routineTime > 10000) {
    sitTest();
  }
  else{
    writetoCard();
    walk();
  }
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
/*
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
}*/

void walk() {
  // Increment each leg's time in the gait cycle 
  uint16_t incrementTime = millis() - lastWalkTime;
  lastWalkTime = millis();
  Serial.println(incrementTime);
  for (int i = 0; i < 4; i ++) {
    legTime[i] = (legTime[i]+incrementTime)%gaitPeriod;
  }
  // Move each leg
    for (int i = 0; i < 4; i+=2) {
      moveFrontLeg(Legs[i],legTime[i]);
    }
    for (int i = 1; i < 4; i+=2) {
      moveBackLeg(Legs[i],legTime[i]);
    }
}

bool bellyRoll(){
  sensors_event_t event;
  bno.getEvent(&event);
  float orientation = (float)event.orientation.z;
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
          Legs[i].m2.moveToDegree(-100);
          Legs[i].m3.moveToDegree(0); 
        }
        rollStep = 1;
      }
  }
  else {
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
  return false;
}

bool backRoll(){
  sensors_event_t event;
  bno.getEvent(&event);
  float orientation = (float)event.orientation.z;
  if (rollStep == 0)
  {
    if (orientation < -160 || orientation > 160)
    {
      for (int i = 1; i < 4;i+=2)
        {
          Legs[i].m2.moveToDegree(35);
          Legs[i].m3.moveToDegree(-30);
        }
        for (int i = 1; i <4; i +=2)
        {
          Legs[i].m2.moveToDegree(100);
          Legs[i].m3.moveToDegree(0); 
        }
        rollStep = 1;
    }
  }
  else
  {
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
  }
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
      Legs[i].m3.moveToDegree(m3Goal1);
    }
    standStep = 3;
  }
  else if (millis()-standTime > 2000 && standStep == 3) {
    for (int i=0;i<4;i++)
    {
      Legs[i].m2.moveToDegree(m2Goal2);
      Legs[i].m2.moveToDegree(m3Goal2);
    }
    standStep = 0;
    return true;
  }
  return false;
}

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

void writetoCard(){
  int Speed,Load;
  myFile.print(millis()); myFile.print(", ");
  for(int i = 0; i < 4; i ++)
  {
    Speed = ax12GetRegister(Legs[i].m1.motorID,38,2);
    Load = ax12GetRegister(Legs[i].m1.motorID,40,2);
    myFile.print(Speed); myFile.print(", ");
    myFile.print(Load); myFile.print(", ");
    
    Speed = ax12GetRegister(Legs[i].m2.motorID,38,2);
    Load = ax12GetRegister(Legs[i].m2.motorID,40,2);
    myFile.print(Speed); myFile.print(", ");
    myFile.print(Load); myFile.print(", ");
    
    Speed = ax12GetRegister(Legs[i].m3.motorID,38,2);
    Load = ax12GetRegister(Legs[i].m3.motorID,40,2);
    myFile.print(Speed); myFile.print(", ");
    myFile.print(Load); myFile.print(", ");
  }
  myFile.print("\n");

  }

