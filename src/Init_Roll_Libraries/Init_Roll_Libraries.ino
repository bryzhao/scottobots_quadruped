// Script to control position of a single leg
// Author: ScottoBots, 4/8/2019

// import ax12 library to send dynamixel commands
#include <ax12.h>
#include "scottoLeg.h"
#include "scottoMotorInterface.h"

scottoLeg Legs[4];

// leg / motor properties
int leg0[] = {1,2,3}; uint16_t leg1Time; //front left
int leg1[] = {7,8,9}; uint16_t leg2Time; //back left
int leg2[] = {12,11,10}; uint16_t leg3Time; //front right
int leg3[] = {4,5,6}; uint16_t leg4Time; //back right
int motorMins[] = {850,859,865,159,864,156,860,157,861,863,861,861}; // 156 -> 861 for motor 12 (backwards)
int motorMaxs[] = {159,159,159,869,158,860,156,863,160,157,156,156}; // NOTE: swapped min and max for 4, 5, 6

void setup(){
   //open serial port
   Serial.begin(9600);
   delay(250);
   Legs[0] = scottoLeg(leg0[0],leg0[1],leg0[2],motorMins[leg0[0]-1],motorMaxs[leg0[0]-1],motorMins[leg0[1]-1],motorMaxs[leg0[1]-1],motorMins[leg0[2]-1],motorMaxs[leg0[2]-1]);
   Legs[1] = scottoLeg(leg1[0],leg1[1],leg1[2],motorMins[leg1[0]-1],motorMaxs[leg1[0]-1],motorMins[leg1[1]-1],motorMaxs[leg1[1]-1],motorMins[leg1[2]-1],motorMaxs[leg1[2]-1]);
   Legs[2] = scottoLeg(leg2[0],leg2[1],leg2[2],motorMins[leg2[0]-1],motorMaxs[leg2[0]-1],motorMins[leg2[1]-1],motorMaxs[leg2[1]-1],motorMins[leg2[2]-1],motorMaxs[leg2[2]-1]);
   Legs[3] = scottoLeg(leg3[0],leg3[1],leg3[2],motorMins[leg3[0]-1],motorMaxs[leg3[0]-1],motorMins[leg3[1]-1],motorMaxs[leg3[1]-1],motorMins[leg3[2]-1],motorMaxs[leg3[2]-1]);
   Serial.println("\nStarting in 2 seconds");
   delay(2000);
   Serial.println("starting...");
}

void loop(){
  int pos_1, pos_2, pos_3; 
  pos_1 = m1.readDegree();
  pos_2 = m2.readDegree();
  pos_3 = m3.readDegree();
  /*
  delay(10);
  pos_1 = digital_to_degree(1,ax12GetRegister(1, 36, 2)); // 36 corresponds to motor position, 2 bytes
  delay(10);
  pos_2 = digital_to_degree(2,ax12GetRegister(2, 36, 2));
  delay(10);
  pos_3 = digital_to_degree(3,ax12GetRegister(3, 36, 2));*/
  Serial.print(pos_1); Serial.print(", ");
  Serial.print(pos_2); Serial.print(", ");
  Serial.print(pos_3); Serial.println(";");
  
}

