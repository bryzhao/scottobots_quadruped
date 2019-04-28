#include "scottoLeg.h"

scottoLeg::scottoLeg() {}

scottoLeg::scottoLeg(int id1, int id2, int id3, int min1, int max1, int min2, int max2, int min3, int max3) {
	m1 = scottoMotorInterface(id1,min1,max1);
	m2 = scottoMotorInterface(id2,min2,max2);
	m3 = scottoMotorInterface(id3,min3,max3);
}

void scottoLeg::moveAllDigital(int command1, int command2, int command3) {
	m1.moveToDigital(command1);
	m2.moveToDigital(command2);
	m3.moveToDigital(command3);
}

void scottoLeg::moveAllDegree(float command1, float command2, float command3) {
	m1.moveToDegree(command1);
	m2.moveToDegree(command2);
	m3.moveToDegree(command3);
}

void scottoLeg::moveAllRadian(float command1, float command2, float command3) {
	m1.moveToRadian(command1);
	m2.moveToRadian(command2);
	m3.moveToRadian(command3);
}

void scottoLeg::relaxAll() {
	m1.relax();
	m2.relax();
	m3.relax();
}

bool scottoLeg::stepAll(){
	return (((m1.step()) && (m2.step())) && (m3.step()));
} 
