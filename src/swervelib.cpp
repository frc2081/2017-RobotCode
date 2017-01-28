/*Copyright 2016 Charles Young*/

#include <algorithm>
#include "swervelib.h"

double degrees_to_radians(double degrees) {
	double radians;
	radians = (degrees * PI)/180;
	return radians;
}

double radians_to_degrees(double radians) {
	double degrees;
	degrees = radians * 180/PI;
	return degrees;
}


//width and length are between the wheels, not the robot's dimensions
swervelib::swervelib(double width, double length) {
	target_WSRF = 0;
	target_WSLF = 0;
	target_WSLB = 0;
	target_WSRB = 0;
	target_WARF = 0;
	target_WALF = 0;
	target_WALB = 0;
	target_WARB = 0;
	currAngRF = 0;
	currAngLF = 0;
	currAngLB = 0;
	currAngRB = 0;
	MAX_WS = 0;

	A = 0;
	B = 0;
	C = 0;
	D = 0;
	//Radius of robot - corner to center
	radius = sqrt(pow(width, 2) + pow(length, 2));
	_width = width;
	_length = length;

	whl = new wheel();
}

void swervelib::calcWheelVect(double mag,
						 double ang,
						 double rotation) {

	centerVecX = mag * cos(degrees_to_radians(ang+90));
	centerVecY = mag * sin(degrees_to_radians(ang+90));

	//Current facing of the robot
	currAng1 = this->whl->angle1;
	currAng2 = this->whl->angle2;
	currAng3 = this->whl->angle3;
	currAng4 = this->whl->angle4;

	//Calculate the wheel motion vectors.
	/*
	 * Wheel 1
	 * 	X - B
	 * 	Y - C
	 * Wheel 2
	 *	X - B
	 *	Y - D
	 * Wheel 3
	 * 	X - A
	 * 	Y - D
	 * Wheel 4
	 * 	X - A
	 * 	Y - C
*/
	A = centerVecX - rotation * (_length/radius);
	B = centerVecX + rotation * (_length/radius);
	C = centerVecY - rotation * (_width/radius);
	D = centerVecY + rotation * (_width/radius);

	//Calculating the wheel speeds
	target_WSRF = sqrt(pow(B, 2) + pow(C, 2));
	target_WSLF = sqrt(pow(B, 2) + pow(D, 2));
	target_WSLB = sqrt(pow(A, 2) + pow(D, 2));
	target_WSRB = sqrt(pow(A, 2) + pow(C, 2));
	MAX_WS = std::max({target_WSRF, target_WSLF, target_WSLB, target_WSRB});

	//Calculating wanted angle of each wheel
	target_WARF = radians_to_degrees(atan2(B, C));
	target_WALF = radians_to_degrees(atan2(B, D));
	target_WALB = radians_to_degrees(atan2(A, D));
	target_WARB = radians_to_degrees(atan2(A, C));
	if (MAX_WS > 1) {
		target_WSRF /= MAX_WS;
		target_WSLF /= MAX_WS;
		target_WSLB /= MAX_WS;
		target_WSRB /= MAX_WS;
	}


	//Set the wheel speeds and angles to be accessed outside of the class
	this->whl->speed1 = target_WSRF;
	this->whl->speed2 = target_WSLF;
	this->whl->speed3 = target_WSLB;
	this->whl->speed4 = target_WSRB;

	this->whl->angle1 = 360 - (target_WARF + 180);
	this->whl->angle2 = 360 - (target_WALF + 180);
	this->whl->angle3 = 360 - (target_WALB + 180);
	this->whl->angle4 = 360 - (target_WARB + 180);
}
