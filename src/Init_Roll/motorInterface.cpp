#include "motorInterface.h"

motorInterface::motorInterface(int ID, int motorMin, int motorMax) {
  motorID = ID;
  minDigital = motorMin;
  maxDigital = motorMax;
}

float motorInterface::digital_to_degree(int digital_read) {
  float minAngle = -abs(maxDigital-minDigital)*0.2929/2.0;
  float maxAngle = abs(maxDigital-minDigital)*0.2929/2.0;
  return ((float)(digital_read-minDigital)*(maxAngle-minAngle)/(maxDigital-minDigital)+minAngle);
}

float motorInterface::degree_to_digital(float angle) {
  float minAngle = -abs(maxDigital-minDigital)*0.2929/2.0;
  float maxAngle = abs(maxDigital-minDigital)*0.2929/2.0;
  return round((angle-minAngle)*(float)(maxDigital-minDigital)/(maxAngle-minAngle)+minDigital);
}

float motorInterface::digital_to_radian(int digital_read) {
  float minAngle = -abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  float maxAngle = abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  return interpolate(digital_read,minDigital,maxDigital,minAngle,maxAngle);
}

float motorInterface::radian_to_digital(float angle) {
  float minAngle = -abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  float maxAngle = abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  return round(interpolate(angle,minAngle,maxAngle,minDigital,maxDigital));
}

void motorInterface::moveToDigital(int command) {
  SetPosition(ID,command);
}

int motorInterface::readDigital() {
  return ax12GetRegister(ID, 36, 2);
}

void motorInterface::moveToDegree(float command) {
  moveToDigital(degree_to_digital(command));
}

void motorInterface::moveToRadian(float command) {
  moveToDigital(radian_to_digital(command));
}

float motorInterface::readDegree() {
  return digital_to_degree(readDigital());
}

float motorInterface::readRadian() {
  return digital_to_radian(readDigital());
}

void relax() {
  Relax(ID);
}

float interpolate(float val,float minIN,float maxIN,float minOUT, float maxOUT){
  return (val-minIN)*(maxOUT-minOUT)/(maxIN-minIN)+minOUT;
}