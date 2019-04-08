// Script to read position of individual motors for one leg
// Author: ScottoBots, 4/8/2019

// import ax12 library to send dynamixel commands
#include <ax12.h>
#include <BioloidController.h>

BioloidController bioloid = BioloidController(1000000);

int id_1, id_2, id_3; // motor id
int pos_1, pos_2, pos_3; // position (absolute)
int IDCheck; 
int RunCheck;

void setup(){
   pinMode(0,OUTPUT);  
   
   //initialize variables 
   id_1 = 1;
   id_2 = 2;
   id_3 = 3;
   pos_1 = 0;
   pos_2 = 0;
   pos_3 = 0;
   
  //open serial port
   Serial.begin(9600);
   delay(500);   
   Serial.println("Starting motor readings...");
}

void loop(){
  Relax(id_1); // relax all motors
  Relax(id_2);
  Relax(id_3);
  delay(5);
  pos_1 = ax12GetRegister(id_1, 36, 2); // 36 corresponds to motor position, 2 bytes
  delay(5);
  pos_2 = ax12GetRegister(id_2, 36, 2);
  delay(5);
  pos_3 = ax12GetRegister(id_3, 36, 2)
  Serial.print(pos_1); Serial.print(", ");
  Serial.print(pos_2); Serial.print(", ");
  Serial.println(pos_3);
}

