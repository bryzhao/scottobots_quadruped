/*
This class allow user to read/ command motor position in either degrees, radians, or digital (raw motor inputs)
When constructing, minDigital is the raw motor reading when the motor is pulled all the way to one side, given mounting bracket constraint (this will correspond to -150 degrees).
maxDigital is the raw motor reading when the motor is pulled all the way to the other side, given the mounting bracket constraint (this will correspond to 150 degrees).
*/

#ifndef scottoMotorInterface_h
#define scottoMotorInterface_h
#include <ax12.h>

class scottoMotorInterface
{
private:
	int minDigital;
	int maxDigital;
	int motorID;
	float movespeed; // speed to move motors when moveslowlyfunction used;
	
	int16_t lastMoveTime;
public:
	bool onStatus; // is the motor powered or not

	float lastCommand; // last command sent to the motor. Should roughly correspond to the motors actual position

        int destination; // motor position to go to slowly
	// Constructure, input ID of motor as well as the motor's min/ max digital values
	scottoMotorInterface(int ID,int motorMin, int motorMax);
	// Constructor with no input
	scottoMotorInterface();
	// Convert digital reading to degree
	float digital_to_degree(int digital_read);
	// Convert degree reading to digital
	float degree_to_digital(float angle);
	// Convert digital reading to digital reading
	float digital_to_radian(int digital_read);
	// Convert radian reading to digital reading
	float radian_to_digital(float angle);
	// Move motor to a desired digital command
	void moveToDigital(int command);
	// Read motor position and retur in digital
	int readDigital();
	// move motor to given degree command
	void moveToDegree(float command);
	// move motor to given radian command
	void moveToRadian(float command);
	// read motor position and return reading in degrees
	float readDegree();
	// read motor and return reading in radians
	float readRadian();
	// relax the motor
	void relax();
	// function for interpolating values
	float interpolate(float val,float minIN,float maxIN,float minOUT, float maxOUT);
	// set speed in digital/milliseconds
	void setSpeed(float speedIn);
	// set destination for motor to move to slowly (in digital commands)
	void setDestinationDigital(int inDest);
	// set destination for motor to move to slowly (in radian commands)
	void setDestinationRadian(float inDest);
	// set destination for motor to move to slowly (in degree commands)
	void setDestinationDegree(float inDest);
	// steps motor at the desired speed towards the destinations. Return true if destination reached
	bool step();
        
};
#endif
