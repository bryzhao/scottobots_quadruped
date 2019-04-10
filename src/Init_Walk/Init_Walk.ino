// Script to control position of a single leg
// Author: ScottoBots, 4/8/2019

// import ax12 library to send dynamixel commands
#include <ax12.h>
#include <BioloidController.h>

BioloidController bioloid = BioloidController(1000000);

int id_1, id_2, id_3; // motor IDs
int pos_1, pos_2, pos_3; // position (absolute)

uint16_t gaitPeriod = 4000;
float motor_1[] = {-0.58511,-0.57556,-0.56298,-0.55017,-0.53715,-0.52392,-0.51049,-0.49685,-0.48301,-0.46897,-0.45472,-0.44027,-0.42562,-0.41078,-0.39574,-0.38051,-0.36509,-0.34948,-0.3337,-0.31775,-0.30162,-0.28533,-0.26888,-0.25229,-0.23555,-0.21867,-0.20167,-0.18454,-0.16731,-0.14998,-0.13255,-0.11505,-0.097468,-0.07983,-0.062143,-0.044416,-0.026661,-0.0088891,0.0088882,0.02666,0.044415,0.062142,0.07983,0.097468,0.11504,0.13255,0.14998,0.16731,0.18454,0.20166,0.21867,0.23554,0.25229,0.26888,0.28533,0.30162,0.31774,0.3337,0.34948,0.36509,0.38051,0.39574,0.41078,0.42562,0.44027,0.45472,0.46897,0.48301,0.49685,0.51049,0.52392,0.53714,0.55017,0.56298,0.57556,0.58511,0.58797,0.588,0.588,0.588,0.588,0.57849,0.52913,0.44752,0.35731,0.2606,0.15866,0.053283,-0.053283,-0.15866,-0.2606,-0.35731,-0.44752,-0.52913,-0.57849,-0.588,-0.588,-0.588,-0.588,-0.58797,-0.58511};
float motor_2[] = {-1.5899,-1.5862,-1.5812,-1.5763,-1.5713,-1.5663,-1.5613,-1.5563,-1.5513,-1.5464,-1.5415,-1.5366,-1.5318,-1.5271,-1.5225,-1.5179,-1.5135,-1.5092,-1.505,-1.5009,-1.497,-1.4933,-1.4897,-1.4863,-1.4831,-1.4801,-1.4772,-1.4746,-1.4722,-1.47,-1.4681,-1.4663,-1.4649,-1.4636,-1.4626,-1.4619,-1.4614,-1.4611,-1.4611,-1.4614,-1.4619,-1.4626,-1.4636,-1.4649,-1.4663,-1.4681,-1.47,-1.4722,-1.4746,-1.4772,-1.4801,-1.4831,-1.4863,-1.4897,-1.4933,-1.497,-1.5009,-1.505,-1.5092,-1.5135,-1.5179,-1.5225,-1.5271,-1.5318,-1.5366,-1.5415,-1.5464,-1.5513,-1.5563,-1.5613,-1.5663,-1.5713,-1.5763,-1.5812,-1.5864,-1.6176,-1.7125,-1.8426,-1.9853,-2.1432,-2.3184,-2.4845,-2.5574,-2.5572,-2.5482,-2.5368,-2.5263,-2.52,-2.52,-2.5263,-2.5368,-2.5482,-2.5572,-2.5574,-2.4845,-2.3184,-2.1432,-1.9853,-1.8426,-1.7125,-1.6176};
float motor_3[] = {2.0135,2.0186,2.0251,2.0313,2.0374,2.0432,2.0488,2.0541,2.0593,2.0642,2.069,2.0735,2.0779,2.082,2.0859,2.0897,2.0932,2.0966,2.0998,2.1028,2.1056,2.1082,2.1107,2.113,2.1151,2.1171,2.1189,2.1206,2.122,2.1234,2.1246,2.1256,2.1265,2.1272,2.1278,2.1282,2.1285,2.1287,2.1287,2.1285,2.1282,2.1278,2.1272,2.1265,2.1256,2.1246,2.1234,2.122,2.1206,2.1189,2.1171,2.1151,2.113,2.1107,2.1082,2.1056,2.1028,2.0998,2.0966,2.0932,2.0897,2.0859,2.082,2.0779,2.0735,2.069,2.0642,2.0593,2.0541,2.0488,2.0432,2.0374,2.0313,2.0251,2.0189,2.0402,2.1228,2.2245,2.3169,2.3981,2.4657,2.5191,2.5722,2.6249,2.6683,2.7014,2.7237,2.7349,2.7349,2.7237,2.7014,2.6683,2.6249,2.5722,2.5191,2.4657,2.3981,2.3169,2.2245,2.1228,2.0402};

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
   timeBetweenGaitPos = (float)gaitPeriod/((float)(sizeof(motor_1)/sizeof(motor_1[0])));   
   Serial.println("\nStarting in 3 seconds");
   //delay(3000);
}

void loop(){
  Serial.println("Starting Walk");
  uint32_t startTime = millis();
  moveLeg(leg1,leg1Time);
  /*delay(10);
  moveLeg(leg2,leg2Time);
  delay(10);
  moveLeg(leg3,leg3Time);
  delay(10);
  moveLeg(leg4,leg4Time);*/
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
  float radCommand = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_1[gaitNum],motor_1[gaitNum+1]);
  int digCommand = radian_to_digital(legMotors[0],radCommand);
  Serial.println(legMotors[0]);
  //SetPosition(digCommand, legMotors[0]);
  delay(10);
  /*
  radCommand = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_2[gaitNum],motor_2[gaitNum+1]);
  digCommand = radian_to_digital(legMotors[1],radCommand);
  SetPosition(digCommand, legMotors[1]);
  delay(10);
  
  radCommand = interpolate(gaitTime,timeBetweenGaitPos*(float)gaitNum,timeBetweenGaitPos*(float)(gaitNum+1),motor_3[gaitNum],motor_3[gaitNum+1]);
  digCommand = radian_to_digital(legMotors[2],radCommand);
  SetPosition(digCommand, legMotors[2]);*/
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
