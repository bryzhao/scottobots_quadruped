// Script to read position and convert to angles
// Author: ScottoBots, 4/8/2019

// import ax12 library to send dynamixel commands
#include <ax12.h>
#include <BioloidController.h>

BioloidController bioloid = BioloidController(1000000);

int id_1, id_2, id_3; // motor id
int pos_1, pos_2, pos_3; // position (absolute)
int IDCheck; 
int RunCheck;
int motorMins[] = {850,859,865,859,158,860,860,157,861,863,861,156};
int motorMaxs[] = {159,159,159,159,864,156,156,863,160,157,156,861};

void setup(){
   pinMode(0,OUTPUT);  
   
   //initialize variables 
   id_1 = 10;
   id_2 = 11;
   id_3 = 12;
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
  pos_3 = ax12GetRegister(id_3, 36, 2);
  Serial.print(pos_1); Serial.print(", ");
  Serial.print(pos_2); Serial.print(", ");
  Serial.println(pos_3);
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
