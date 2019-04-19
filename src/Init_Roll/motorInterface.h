#ifndef MOTORINTERFACE_H
#define MOTORINTERFACE_H

#include <ax12.h>

class motorInterface
{
private:
	int minDigital;
	int minDigital;
	int motorID;
public:
	motorInterface(int ID,int motorMin, int motorMax);
	float digital_to_degree(int digital_read);
	float degree_to_digital(float angle);
	float digital_to_radian(int digital_read);
	float radian_to_digital(float angle);
	void moveToDigital(int command);
	int readDigital();
	void moveToDegree(float command);
	void moveToRadian(float command);
	float readDegree();
	float readRadian();
	void relax();
	float interpolate();
}
