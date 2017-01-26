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
	target_WS1 = 0;
	target_WS2 = 0;
	target_WS3 = 0;
	target_WS4 = 0;
	target_WA1 = 0;
	target_WA2 = 0;
	target_WA3 = 0;
	target_WA4 = 0;
	currAng1 = 0;
	currAng2 = 0;
	currAng3 = 0;
	currAng4 = 0;
	MAX_WS = 0;

	A = 0;
	B = 0;
	C = 0;
	D = 0;
	//Radius of robot - corner to center
	R = sqrt(pow(width, 2) + pow(length, 2));
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
	A = centerVecX - rotation * (_length/R);
	B = centerVecX + rotation * (_length/R);
	C = centerVecY - rotation * (_width/R);
	D = centerVecY + rotation * (_width/R);

	//Calculating the wheel speeds
	target_WS1 = sqrt(pow(B, 2) + pow(C, 2));
	target_WS2 = sqrt(pow(B, 2) + pow(D, 2));
	target_WS3 = sqrt(pow(A, 2) + pow(D, 2));
	target_WS4 = sqrt(pow(A, 2) + pow(C, 2));
	MAX_WS = std::max({target_WS1, target_WS2, target_WS3, target_WS4});

	//Calculating wanted angle of each wheel
	target_WA1 = radians_to_degrees(atan2(B, C));
	target_WA2 = radians_to_degrees(atan2(B, D));
	target_WA3 = radians_to_degrees(atan2(A, D));
	target_WA4 = radians_to_degrees(atan2(A, C));
	if (MAX_WS > 1) {
		target_WS1 /= MAX_WS;
		target_WS2 /= MAX_WS;
		target_WS3 /= MAX_WS;
		target_WS4 /= MAX_WS;
	}


	//Set the wheel speeds and angles to be accessed outside of the class
	this->whl->speed1 = target_WS1;
	this->whl->speed2 = target_WS2;
	this->whl->speed3 = target_WS3;
	this->whl->speed4 = target_WS4;

	this->whl->angle1 = 360 - (target_WA1 + 180);
	this->whl->angle2 = 360 - (target_WA2 + 180);
	this->whl->angle3 = 360 - (target_WA3 + 180);
	this->whl->angle4 = 360 - (target_WA4 + 180);
}
