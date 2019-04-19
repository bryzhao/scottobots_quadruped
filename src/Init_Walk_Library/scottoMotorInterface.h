#ifndef scottoMotorInterface_h
#define scottoMotorInterface_h
#include <ax12.h>

class scottoMotorInterface
{
private:
	int minDigital;
	int maxDigital;
	int motorID;
public:
	scottoMotorInterface(int ID,int motorMin, int motorMax);
        scottoMotorInterface();
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
	float interpolate(float val,float minIN,float maxIN,float minOUT, float maxOUT);
};
#endif
