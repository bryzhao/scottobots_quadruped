// Script to control position of a single leg
// Author: ScottoBots, 4/8/2019

// import ax12 library to send dynamixel commands
#include <ax12.h>
#include <BioloidController.h>

BioloidController bioloid = BioloidController(1000000);

int id_1, id_2, id_3; // motor IDs
int pos_1, pos_2, pos_3; // position (absolute)

uint16_t gaitPeriod = 1000;
float motor_1[] = {-0.7823,-0.7803,-0.7779,-0.7751,-0.7719,-0.7685,-0.7650,-0.7823};
float motor_2[] = {-1.7117,-1.7119,-1.7122,-1.7126,-1.7129,-1.7133,-1.7136,-1.7117};
float motor_3[] = {1.6330,1.6356,1.6389,1.6427,1.6470,1.6514,1.6558,1.6330};
float timeBetweenGaitPos;

int leg1[] = {1,2,3}; uint16_t leg1Time; //front left
int leg2[] = {4,5,6}; uint16_t leg2Time; //back left
int leg3[] = {7,8,9}; uint16_t leg3Time; //back right
int leg4[] = {10,11,12}; uint16_t leg4Time; //front right
int motorMins[] = {850,859,865,859,158,860,860,157,861,863,861,156};
int motorMaxs[] = {159,159,159,159,864,156,156,863,160,157,156,861};

void setup(){
   //open serial port
   Serial.begin(9600);
   delay(500);
   
   //Configure leg time offset
   leg4Time = 0;
   leg3Time = gaitPeriod/4;
   leg2Time = gaitPeriod/2;
   leg1Time = gaitPeriod*3/4;
   float timeBetweenGaitPos = (float)gaitPeriod/((float)(sizeof(motor_1)/sizeof(motor_1[0])));
   
   Serial.println("\nStarting in 3 seconds");
   delay(3000);
}

void loop(){
  Serial.println("Starting Walk");
  uint32_t startTime = millis();
  moveLeg(leg1,leg1Time);
  delay(10);
  moveLeg(leg2,leg2Time);
  delay(10);
  moveLeg(leg3,leg3Time);
  delay(10);
  moveLeg(leg4,leg4Time);
  delay(10);
  int incrementTime = millis() - startTime;
  leg1Time = (leg1Time+incrementTime)%gaitPeriod;
  leg2Time = (leg2Time+incrementTime)%gaitPeriod;
  leg3Time = (leg3Time+incrementTime)%gaitPeriod;
  leg4Time = (leg4Time+incrementTime)%gaitPeriod;
}

void moveLeg(int* legMotors, uint16_t gaitTime)
{
  int gaitNum = floor((float)gaitTime/timeBetweenGaitPos);
  SetPosition(interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_1[gaitNum],motor_1[gaitNum+1]), legMotors[0]);
  delay(10);
  SetPosition(interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_2[gaitNum],motor_2[gaitNum+1]), legMotors[1]);
  delay(10);
  SetPosition(interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_3[gaitNum],motor_3[gaitNum+1]), legMotors[2]);
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
  return (val-minIN)*(maxOUT-minOUT)/(maxIN-maxIN)+minIN;
}
