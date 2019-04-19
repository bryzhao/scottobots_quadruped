#include "scottoMotorInterface.h"
#ifndef scottoLeg_h
#define scottoLeg_h

class scottoLeg
{
public:
	scottoMotorInterface m1;
	scottoMotorInterface m2;
	scottoMotorInterface m3;
	scottoLeg();
	scottoLeg(int id1, int id2, int id3, int min1, int max1, int min2, int max2, int min3, int max3);
	void moveAllDigital(int command1, int command2, int command3);
	void moveAllDegree(float command1, float command2, float command3);
	void moveAllRadian(float command1, float command2, float command3);
        void relaxAll();
};
#endif
