/*
This class allows for storage/ control of all motors on one leg.
When constructing, min is the raw motor reading when the motor is pulled all the way to one side, given mounting bracket constraint (this will correspond to -150 degrees).
max is the raw motor reading when the motor is pulled all the way to the other side, given the mounting bracket constraint (this will correspond to 150 degrees)
*/

#include "scottoMotorInterface.h"
#ifndef scottoLeg_h
#define scottoLeg_h

class scottoLeg
{
public:
        // Top motor on the leg
	scottoMotorInterface m1;
        // Middle motor on the leg
	scottoMotorInterface m2;
        // Bottom motor on the leg
	scottoMotorInterface m3;
        // constructor with no parameters
	scottoLeg();
        // Constructers that take all the IDs of the motors as well as their min/ max digital values
	scottoLeg(int id1, int id2, int id3, int min1, int max1, int min2, int max2, int min3, int max3);
        // Move all motors in digital command
	void moveAllDigital(int command1, int command2, int command3);
        // Move all motors in degree command
	void moveAllDegree(float command1, float command2, float command3);
        // Move all motors in radian commands
	void moveAllRadian(float command1, float command2, float command3);
        // Relax all motors
        void relaxAll();
};
#endif
