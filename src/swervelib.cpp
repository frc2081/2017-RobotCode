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

swervelib::swervelib(double width, double length) {
	target_WS1 = 0;
	target_WS2 = 0;
	target_WS3 = 0;
	target_WS4 = 0;
	MAX_WS = 0;

	A = 0;
	B = 0;
	C = 0;
	D = 0;
	R = sqrt(pow(width, 2) + pow(width, 2));
	_width = width;
	_length = length;

	whl = new wheel();
}

void swervelib::calcWheelVect(double x,
						 double y,
						 double rudder) {

	A = x - rudder * (_length/R);
	B = x + rudder * (_length/R);
	C = y - rudder * (_width/R);
	D = y + rudder * (_width/R);

	target_WS1 = sqrt(pow(B, 2) + pow(C, 2));
	target_WS2 = sqrt(pow(B, 2) + pow(D, 2));
	target_WS3 = sqrt(pow(A, 2) + pow(D, 2));
	target_WS4 = sqrt(pow(A, 2) + pow(C, 2));
	MAX_WS = std::max({target_WS1, target_WS2, target_WS3, target_WS4});

	target_WA1 = atan2(B, C) * 180./PI;
	target_WA2 = atan2(B, D) * 180./PI;
	target_WA3 = atan2(A, D) * 180./PI;
	target_WA4 = atan2(A, C) * 180./PI;
	if (MAX_WS > 1) {
		target_WS1 /= MAX_WS;
		target_WS2 /= MAX_WS;
		target_WS3 /= MAX_WS;
		target_WS4 /= MAX_WS;
	}

	this->whl->speed1 = target_WS1;
	this->whl->speed2 = target_WS2;
	this->whl->speed3 = target_WS3;
	this->whl->speed4 = target_WS4;

	this->whl->angle1 = 360 - (target_WA1 + 180);
	this->whl->angle2 = 360 - (target_WA2 + 180);
	this->whl->angle3 = 360 - (target_WA3 + 180);
	this->whl->angle4 = 360 - (target_WA4 + 180);
}
