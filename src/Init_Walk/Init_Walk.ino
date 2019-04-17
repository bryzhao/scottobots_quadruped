// Script to control position of a single leg
// Author: ScottoBots, 4/8/2019

// import ax12 library to send dynamixel commands
#include <ax12.h>
#include <BioloidController.h>

BioloidController bioloid = BioloidController(1000000);

//Length of Gait
uint16_t gaitPeriod = 2000;
//Trajectory of the three motors on each leg
float motor_1Front[] = {42,42,41,41,40,39,38,37,35,34,33,31,30,28,27,24,23,21,19,17,15,13,11,9,6,3,0,-1,-2,-2,-1.2497,1.2221,5.2152,10.4061,16.3742,22.636,28.6843,34.0291,38.2373,40.968};
float motor_2Front[] = {-27,-27,-27,-27,-27,-28,-27,-27,-27,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-20.7816,-15.0265,-10.2009,-6.6958,-4.7951,-4.6528,-6.2804,-9.5461,-14.1853,-19.8221};
float motor_3Front[] = {-41,-41,-43,-44,-46,-47,-48,-50,-52,-53,-55,-57,-58,-60,-61,-62,-63,-64,-64,-65,-65,-66,-66,-66,-67,-67,-67,-67,-67,-67,-67,-64.1111,-61.2222,-58.3333,-55.4444,-52.5556,-49.6667,-46.7778,-43.8889,-41};
float motor_1Back[] = {-40.968,-38.2373,-34.0291,-28.6843,-22.636,-16.3742,-10.4061,-5.2152,-1.2221,1.2497,2,2,1,0,-3,-6,-9,-11,-13,-15,-17,-19,-21,-23,-24,-27,-28,-30,-31,-33,-34,-35,-37,-38,-39,-40,-41,-41,-42,-42};
float motor_2Back[] = {-19.8221,-14.1853,-9.5461,-6.2804,-4.6528,-4.7951,-6.6958,-10.2009,-15.0265,-20.7816,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-26,-27,-27,-27,-28,-27,-27,-27,-27,-27};
float motor_3Back[] = {-41,-43.8889,-46.7778,-49.6667,-52.5556,-55.4444,-58.3333,-61.2222,-64.1111,-67,-67,-67,-67,-67,-67,-67,-66,-66,-66,-65,-65,-64,-64,-63,-62,-61,-60,-58,-57,-55,-53,-52,-50,-48,-47,-46,-44,-43,-41,-41};

// float motor_1[] = {-30,-30,-30,-10,10,30,30,30,10,-10,-30};//{-0.29256,-0.28778,-0.28149,-0.27508,-0.26857,-0.26196,-0.25524,-0.24843,-0.24151,-0.23448,-0.22736,-0.22014,-0.21281,-0.20539,-0.19787,-0.19025,-0.18254,-0.17474,-0.16685,-0.15887,-0.15081,-0.14266,-0.13444,-0.12614,-0.11777,-0.10933,-0.10083,-0.092272,-0.083655,-0.074988,-0.066276,-0.057523,-0.048734,-0.039915,-0.031071,-0.022208,-0.01333,-0.0044445,0.0044441,0.01333,0.022207,0.031071,0.039915,0.048734,0.057522,0.066276,0.074988,0.083655,0.092271,0.10083,0.10933,0.11777,0.12614,0.13444,0.14266,0.15081,0.15887,0.16685,0.17474,0.18254,0.19025,0.19787,0.20539,0.21281,0.22014,0.22736,0.23448,0.24151,0.24843,0.25524,0.26196,0.26857,0.27508,0.28149,0.28778,0.29256,0.29399,0.294,0.294,0.294,0.294,0.28925,0.26456,0.22376,0.17865,0.1303,0.079328,0.026642,-0.026641,-0.079328,-0.1303,-0.17865,-0.22376,-0.26456,-0.28925,-0.294,-0.294,-0.294,-0.294,-0.29399,-0.29256};
// float motor_2[] = {-30,0,30,30,30,30,0,-30,-30,-30,-30};//{-0.79495,-0.79308,-0.79062,-0.78814,-0.78564,-0.78314,-0.78064,-0.77815,-0.77566,-0.77319,-0.77074,-0.76831,-0.76591,-0.76355,-0.76124,-0.75897,-0.75675,-0.75459,-0.7525,-0.75047,-0.74852,-0.74664,-0.74485,-0.74315,-0.74154,-0.74003,-0.73861,-0.7373,-0.7361,-0.73501,-0.73403,-0.73317,-0.73243,-0.73181,-0.73131,-0.73093,-0.73068,-0.73056,-0.73056,-0.73068,-0.73093,-0.73131,-0.73181,-0.73243,-0.73317,-0.73403,-0.73501,-0.7361,-0.7373,-0.73861,-0.74003,-0.74154,-0.74315,-0.74485,-0.74664,-0.74852,-0.75047,-0.7525,-0.75459,-0.75675,-0.75897,-0.76124,-0.76355,-0.76591,-0.76831,-0.77074,-0.77319,-0.77566,-0.77815,-0.78064,-0.78314,-0.78564,-0.78814,-0.79062,-0.79321,-0.8088,-0.85624,-0.9213,-0.99263,-1.0716,-1.1592,-1.2422,-1.2787,-1.2786,-1.2741,-1.2684,-1.2631,-1.26,-1.26,-1.2631,-1.2684,-1.2741,-1.2786,-1.2787,-1.2422,-1.1592,-1.0716,-0.99264,-0.9213,-0.85624,-0.8088};
// float motor_3[] = {60,60,60,60,60,60,60,60,60,60,60};//{1.0067,1.0093,1.0125,1.0157,1.0187,1.0216,1.0244,1.0271,1.0296,1.0321,1.0345,1.0368,1.0389,1.041,1.043,1.0448,1.0466,1.0483,1.0499,1.0514,1.0528,1.0541,1.0553,1.0565,1.0576,1.0585,1.0595,1.0603,1.061,1.0617,1.0623,1.0628,1.0632,1.0636,1.0639,1.0641,1.0643,1.0643,1.0643,1.0643,1.0641,1.0639,1.0636,1.0632,1.0628,1.0623,1.0617,1.061,1.0603,1.0595,1.0585,1.0576,1.0565,1.0553,1.0541,1.0528,1.0514,1.0499,1.0483,1.0466,1.0448,1.043,1.041,1.0389,1.0368,1.0345,1.0321,1.0296,1.0271,1.0244,1.0216,1.0187,1.0157,1.0125,1.0094,1.0201,1.0614,1.1122,1.1584,1.1991,1.2329,1.2595,1.2861,1.3125,1.3341,1.3507,1.3618,1.3675,1.3675,1.3618,1.3507,1.3341,1.3125,1.2861,1.2595,1.2329,1.1991,1.1584,1.1122,1.0614,1.0201};

float timeBetweenGaitPos;

// Motor Ids for each leg
int leg1[] = {1,2,3}; uint16_t leg1Time; //front left
int leg2[] = {7,8,9}; uint16_t leg2Time; //back left
int leg3[] = {12,11,10}; uint16_t leg3Time; //front right
int leg4[] = {4,5,6}; uint16_t leg4Time; //back right
// int motorMins[] = {850,859,865,859,158,860,860,157,861,863,861,861}; // 156 -> 861 for motor 12 (backwards)
// int motorMaxs[] = {159,159,159,159,864,156,156,863,160,157,156,156}; // NOTE: swapped min and max for 4, 5, 6

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
  /*int pos_1, pos_2, pos_3;
  delay(10);
  pos_1 = digital_to_degree(1,ax12GetRegister(1, 36, 2)); // 36 corresponds to motor position, 2 bytes
  delay(10);
  pos_2 = digital_to_degree(2,ax12GetRegister(2, 36, 2));
  delay(10);
  pos_3 = digital_to_degree(3,ax12GetRegister(3, 36, 2));
  Serial.print(pos_1); Serial.print(", ");
  Serial.print(pos_2); Serial.print(", ");
  Serial.print(pos_3); Serial.println(";");
  */
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
  leg4Time = (leg4Time+incrementTime)%gaitPeriod;
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
