// Script to read position of individual motors
// Author: ScottoBots, 4/3/2019

// import ax12 library to send dynamixel commands
#include <ax12.h>
#include <BioloidController.h>

BioloidController bioloid = BioloidController(1000000);

int id; // motor id
int pos; // position (absolute)
int IDCheck; 
int RunCheck;

void setup(){
   pinMode(0,OUTPUT);  
   
   //initialize variables 
   id = 9;
   pos = 0;
   
  //open serial port
   Serial.begin(9600);
   delay(500);   
}

void loop(){
  pos =  ax12GetRegister(id, 36, 2); // 36 corresponds to motor position, 2 bytes
  Relax(id);
  Serial.println(pos);
}

