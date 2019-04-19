// Script to control position of a single leg
// Author: ScottoBots, 4/8/2019

// import ax12 library to send dynamixel commands
#include <ax12.h>

//Length of Gait
uint16_t gaitPeriod = 2000;

//Trajectory of the three motors on each leg
float motor_1Front[] = {42,42,41,41,40,39,38,37,35,34,33,31,30,28,27,24,23,21,19,17,15,13,11,9,6,3,0,-1,-2,-2,-1.2497,1.2221,5.2152,10.4061,16.3742,22.636,28.6843,34.0291,38.2373,40.968};
float motor_2Front[] = {-27,-27,-27,-27,-27,-28,-27,-27,-27,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-20.7816,-15.0265,-10.2009,-6.6958,-4.7951,-4.6528,-6.2804,-9.5461,-14.1853,-19.8221};
float motor_3Front[] = {-41,-41,-43,-44,-46,-47,-48,-50,-52,-53,-55,-57,-58,-60,-61,-62,-63,-64,-64,-65,-65,-66,-66,-66,-67,-67,-67,-67,-67,-67,-67,-64.1111,-61.2222,-58.3333,-55.4444,-52.5556,-49.6667,-46.7778,-43.8889,-41};
float motor_1Back[] = {-40.968,-38.2373,-34.0291,-28.6843,-22.636,-16.3742,-10.4061,-5.2152,-1.2221,1.2497,2,2,1,0,-3,-6,-9,-11,-13,-15,-17,-19,-21,-23,-24,-27,-28,-30,-31,-33,-34,-35,-37,-38,-39,-40,-41,-41,-42,-42};
float motor_2Back[] = {-19.8221,-14.1853,-9.5461,-6.2804,-4.6528,-4.7951,-6.6958,-10.2009,-15.0265,-20.7816,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-27,-27,-27,-28,-27,-27,-27,-27,-27};
float motor_3Back[] = {-41,-43.8889,-46.7778,-49.6667,-52.5556,-55.4444,-58.3333,-61.2222,-64.1111,-67,-67,-67,-67,-67,-67,-67,-66,-66,-66,-65,-65,-64,-64,-63,-62,-61,-60,-58,-57,-55,-53,-52,-50,-48,-47,-46,-44,-43,-41,-41};


float timeBetweenGaitPos;

// Motor Ids for each leg
int leg1[] = {1,2,3}; uint16_t leg1Time; //front left
int leg2[] = {7,8,9}; uint16_t leg2Time; //back left
int leg3[] = {12,11,10}; uint16_t leg3Time; //front right
int leg4[] = {4,5,6}; uint16_t leg4Time; //back right

int motorMins[] = {850,859,865,159,864,156,860,157,861,863,861,861}; // 156 -> 861 for motor 12 (backwards)
int motorMaxs[] = {159,159,159,869,158,860,156,863,160,157,156,156}; // NOTE: swapped min and max for 4, 5, 6

void setup(){
   //open serial port
   Serial.begin(9600);
   delay(500);
   
   // Stagger the time that the legs start at in the gait cycle
   leg1Time = 0;
   leg3Time = gaitPeriod/4;
   leg2Time = gaitPeriod/2;
   leg4Time = gaitPeriod*3/4;
   
   // Compute the time between each trajectory value
   timeBetweenGaitPos = (float)gaitPeriod/((float)(sizeof(motor_1Front)/sizeof(motor_1Front[0])-1));   
   
   Serial.println("\nStarting in 2 seconds");
   Relax(1);
   Relax(2);
   Relax(3);
   delay(2000);
   Serial.println("starting...");
}

void loop(){
  int pos_1, pos_2, pos_3; 
  delay(10);
  pos_1 = digital_to_degree(1,ax12GetRegister(1, 36, 2)); // 36 corresponds to motor position, 2 bytes
  delay(10);
  pos_2 = digital_to_degree(2,ax12GetRegister(2, 36, 2));
  delay(10);
  pos_3 = digital_to_degree(3,ax12GetRegister(3, 36, 2));
  Serial.print(pos_1); Serial.print(", ");
  Serial.print(pos_2); Serial.print(", ");
  Serial.print(pos_3); Serial.println(";");
  
  /*
  int digCommand = degree_to_digital(1,-85);
  SetPosition(1,digCommand);
  digCommand = degree_to_digital(2,100);
  SetPosition(2,digCommand);
  digCommand = degree_to_digital(3,100);
  SetPosition(3,digCommand);
  
  /*int digCommand = degree_to_digital(1,-85);
  SetPosition(1,digCommand);
  digCommand = degree_to_digital(2,70);
  SetPosition(2,digCommand);
  digCommand = degree_to_digital(3,-84);
  SetPosition(3,digCommand);
  
  digCommand = degree_to_digital(7,85);
  SetPosition(7,digCommand);
  digCommand = degree_to_digital(8,-70);
  SetPosition(8,digCommand);
  digCommand = degree_to_digital(9,84);
  SetPosition(9,digCommand);

  digCommand = degree_to_digital(12,-85);
  SetPosition(12,digCommand);
  digCommand = degree_to_digital(11,70);
  SetPosition(11,digCommand);
  digCommand = degree_to_digital(10,-84);
  SetPosition(10,digCommand);
  
  digCommand = degree_to_digital(4,85);
  SetPosition(4,digCommand);
  digCommand = degree_to_digital(5,-70);
  SetPosition(5,digCommand);
  digCommand = degree_to_digital(6,84);
  SetPosition(6,digCommand);*/
  /*
  uint32_t startTime = millis();
  
  // Move each leg
  moveFrontLeg(leg1,leg1Time);
  moveBackLeg(leg2,leg2Time);
  moveFrontLeg(leg3,leg3Time);
  moveBackLeg(leg4,leg4Time);
 
 // Increment each leg's time in the gait cycle 
  int incrementTime = millis() - startTime;
  leg1Time = (leg1Time+incrementTime)%gaitPeriod;
  leg2Time = (leg2Time+incrementTime)%gaitPeriod;
  leg3Time = (leg3Time+incrementTime)%gaitPeriod;
  leg4Time = (leg4Time+incrementTime)%gaitPeriod;*/
}


// Move the given leg
void moveFrontLeg(int* legMotors, uint16_t gaitTime)
{
  int gaitNum = floor((float)gaitTime/timeBetweenGaitPos);
  Serial.println(gaitNum);
  float radCommand = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_1Front[gaitNum],motor_1Front[gaitNum+1]);
  int digCommand = degree_to_digital(legMotors[0],radCommand);
  SetPosition(legMotors[0],digCommand);
  delay(1);
  
  radCommand = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_2Front[gaitNum],motor_2Front[gaitNum+1]);
  digCommand = degree_to_digital(legMotors[1],radCommand);
  SetPosition(legMotors[1],digCommand);
  delay(1);
  
  radCommand = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_3Front[gaitNum],motor_3Front[gaitNum+1]);
  digCommand = degree_to_digital(legMotors[2],radCommand);
  SetPosition(legMotors[2],digCommand);
  delay(1);
}

void moveBackLeg(int* legMotors, uint16_t gaitTime)
{
  int gaitNum = floor((float)gaitTime/timeBetweenGaitPos);
  Serial.println(gaitNum);
  float radCommand = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_1Back[gaitNum],motor_1Back[gaitNum+1]);
  int digCommand = degree_to_digital(legMotors[0],radCommand);
  SetPosition(legMotors[0],digCommand);
  delay(1);
  
  radCommand = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_2Back[gaitNum],motor_2Back[gaitNum+1]);
  digCommand = degree_to_digital(legMotors[1],radCommand);
  SetPosition(legMotors[1],digCommand);
  delay(1);
  
  radCommand = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_3Back[gaitNum],motor_3Back[gaitNum+1]);
  digCommand = degree_to_digital(legMotors[2],radCommand);
  SetPosition(legMotors[2],digCommand);
  delay(1);
}

double digital_to_angle(int motor_id, int digital_read) {
  double minDigital = motorMins[motor_id-1];
  double maxDigital = motorMaxs[motor_id-1];
  double minAngle = -abs(maxDigital-minDigital)*0.2929/2.0;
  double maxAngle = abs(maxDigital-minDigital)*0.2929/2.0;
  return ((double)(digital_read-minDigital)*(maxAngle-minAngle)/(maxDigital-minDigital)+minAngle);
}

double angle_to_digital(int motor_id, double angle) {
  double minDigital = motorMins[motor_id-1];
  double maxDigital = motorMaxs[motor_id-1];
  double minAngle = -abs(maxDigital-minDigital)*0.2929/2.0;
  double maxAngle = abs(maxDigital-minDigital)*0.2929/2.0;
  return round((angle-minAngle)*(double)(maxDigital-minDigital)/(maxAngle-minAngle)+minDigital);
}

float digital_to_degree(int motor_id, int digital_read) {
  float minDigital = motorMins[motor_id-1];
  float maxDigital = motorMaxs[motor_id-1];
  float minAngle = -abs(maxDigital-minDigital)*0.2929/2.0;
  float maxAngle = abs(maxDigital-minDigital)*0.2929/2.0;
  return ((float)(digital_read-minDigital)*(maxAngle-minAngle)/(maxDigital-minDigital)+minAngle);
}

float degree_to_digital(int motor_id, float angle) {
  float minDigital = motorMins[motor_id-1];
  float maxDigital = motorMaxs[motor_id-1];
  float minAngle = -abs(maxDigital-minDigital)*0.2929/2.0;
  float maxAngle = abs(maxDigital-minDigital)*0.2929/2.0;
  return round((angle-minAngle)*(float)(maxDigital-minDigital)/(maxAngle-minAngle)+minDigital);
}

float digital_to_radian(int motor_id, int digital_read) {
  float minDigital = motorMins[motor_id-1];
  float maxDigital = motorMaxs[motor_id-1];
  float minAngle = -abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  float maxAngle = abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  return interpolate(digital_read,minDigital,maxDigital,minAngle,maxAngle);
}

float radian_to_digital(int motor_id, float angle) {
  float minDigital = motorMins[motor_id-1];
  float maxDigital = motorMaxs[motor_id-1];
  float minAngle = -abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  float maxAngle = abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  return round(interpolate(angle,minAngle,maxAngle,minDigital,maxDigital));
}

float interpolate(float val,float minIN,float maxIN,float minOUT, float maxOUT){
  return (val-minIN)*(maxOUT-minOUT)/(maxIN-minIN)+minOUT;
}
