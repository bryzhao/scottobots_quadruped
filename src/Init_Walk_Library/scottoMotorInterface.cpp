#include <EncodersAB.h>

#include "scottoMotorInterface.h"

scottoMotorInterface::scottoMotorInterface(){
}

scottoMotorInterface::scottoMotorInterface(int ID, int motorMin, int motorMax) {
  motorID = ID;
  minDigital = motorMin;
  maxDigital = motorMax;
  relax();
}

float scottoMotorInterface::digital_to_degree(int digital_read) {
  float minAngle = -abs(maxDigital-minDigital)*0.2929/2.0;
  float maxAngle = abs(maxDigital-minDigital)*0.2929/2.0;
  return ((float)(digital_read-minDigital)*(maxAngle-minAngle)/(maxDigital-minDigital)+minAngle);
}

float scottoMotorInterface::degree_to_digital(float angle) {
  float minAngle = -abs(maxDigital-minDigital)*0.2929/2.0;
  float maxAngle = abs(maxDigital-minDigital)*0.2929/2.0;
  return round((angle-minAngle)*(float)(maxDigital-minDigital)/(maxAngle-minAngle)+minDigital);
}

float scottoMotorInterface::digital_to_radian(int digital_read) {
  float minAngle = -abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  float maxAngle = abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  return interpolate(digital_read,minDigital,maxDigital,minAngle,maxAngle);
}

float scottoMotorInterface::radian_to_digital(float angle) {
  float minAngle = -abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  float maxAngle = abs(maxDigital-minDigital)*(300*PI/180/1024)/2.0;
  return round(interpolate(angle,minAngle,maxAngle,minDigital,maxDigital));
}

void scottoMotorInterface::moveToDigital(int command) {
  SetPosition(motorID,command);
  onStatus = true;
  lastCommand = command;
  lastMoveTime = millis();
  delay(1);
}

int scottoMotorInterface::readDigital() {
  return ax12GetRegister(motorID, 36, 2);
}

void scottoMotorInterface::moveToDegree(float command) {
  moveToDigital(degree_to_digital(command));
}

void scottoMotorInterface::moveToRadian(float command) {
  moveToDigital(radian_to_digital(command));
}

float scottoMotorInterface::readDegree() {
  return digital_to_degree(readDigital());
}

float scottoMotorInterface::readRadian() {
  return digital_to_radian(readDigital());
}

void scottoMotorInterface::relax() {
  Relax(motorID);
  onStatus = false;
}

float scottoMotorInterface::interpolate(float val,float minIN,float maxIN,float minOUT, float maxOUT){
  return (val-minIN)*(maxOUT-minOUT)/(maxIN-minIN)+minOUT;
}

void scottoMotorInterface::setSpeed(float speedIn){
  movespeed = speedIn;
}

void scottoMotorInterface::setDestinationDigital(int inDest){
  destination = inDest;
  lastMoveTime = millis();
}

void scottoMotorInterface::setDestinationDegree(float inDest){
  destination = degree_to_digital(inDest);
  lastMoveTime = millis();
}

void scottoMotorInterface::setDestinationRadian(float inDest){
  destination = radian_to_digital(inDest);
  lastMoveTime = millis();
}

bool scottoMotorInterface::step(){
  float tol = 0.01;
  if (!onStatus){
    lastCommand = readDigital();
    lastMoveTime = millis();
  }
  uint16_t moveTime = millis() - lastMoveTime;
  lastMoveTime = millis();
  float commandChange = constrain((float)destination - lastCommand,-movespeed*(float)moveTime,movespeed*(float)moveTime);
  lastCommand = lastCommand+commandChange;
  moveToDigital(round(lastCommand));
  if (abs(lastCommand-destination) < tol)
    return true;
  else
    return false;
}
