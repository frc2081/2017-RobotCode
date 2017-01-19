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
	R = sqrt(pow(width, 2) + pow(length, 2));
	_width = width;
	_length = length;

	whl = new wheel();
}

void swervelib::calcWheelVect(double x,
						 double y,
						 double rudder) {

	currAng1 = this->whl->angle1;
	currAng2 = this->whl->angle2;
	currAng3 = this->whl->angle3;
	currAng4 = this->whl->angle4;

	A = x - rudder * (_length/R);
	B = x + rudder * (_length/R);
	C = y - rudder * (_width/R);
	D = y + rudder * (_width/R);

	target_WS1 = sqrt(pow(B, 2) + pow(C, 2));
	target_WS2 = sqrt(pow(B, 2) + pow(D, 2));
	target_WS3 = sqrt(pow(A, 2) + pow(D, 2));
	target_WS4 = sqrt(pow(A, 2) + pow(C, 2));
	MAX_WS = std::max({target_WS1, target_WS2, target_WS3, target_WS4});

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

	/* x is the number you compare distance from
	// a is the array of ints
	// count is array size

	for (int i = 1; i < 11; ++i) {
		wheelStableize1 = abs(a[i] - currAng1) < abs(wheelStableize1 - currAng1) ? a[i] : wheelStableize1;
	}
	for (int i = 1; i < 11; ++i) {
		wheelStableize2 = abs(a[i] - currAng1) < abs(wheelStableize2 - currAng1) ? a[i] : wheelStableize2;
	}
	for (int i = 1; i < 11; ++i) {
		wheelStableize3 = abs(a[i] - currAng1) < abs(wheelStableize3 - currAng1) ? a[i] : wheelStableize3;
	}
	for (int i = 1; i < 11; ++i) {
		wheelStableize4 = abs(a[i] - currAng1) < abs(wheelStableize4 - currAng1) ? a[i] : wheelStableize4;
	}

	if(fabs(x) > 0.05 || fabs(y) > 0.05 || fabs(rudder) > 0.05) {
		if (fabs(target_WS1 - wheelStableize1) > 90 && fabs(target_WS1 - wheelStableize1) < 270) {
			target_WS1 = ((int)target_WS1 + 180) % 360;
			target_WS1 *= -1;
		}

		if (fabs(target_WS2 - wheelStableize2) > 90 && fabs(target_WS2 - wheelStableize2) < 270) {
			target_WS2 = ((int)target_WS2 + 180) % 360;
			target_WS2 *= -1;
		}

		if (fabs(target_WS3 - wheelStableize3) > 90 && fabs(target_WS3 - wheelStableize3) < 270) {
			target_WS3 = ((int)target_WS3 + 180) % 360;
			target_WS3 *= -1;
		}

		if (fabs(target_WS4 - wheelStableize4) > 90 && fabs(target_WS4 - wheelStableize4) < 270) {
			target_WS4 = ((int)target_WS4 + 180) % 360;
			target_WS4 *= -1;
		}
	}
	*/
	this->whl->speed1 = target_WS1;
	this->whl->speed2 = target_WS2;
	this->whl->speed3 = target_WS3;
	this->whl->speed4 = target_WS4;

	this->whl->angle1 = 360 - (target_WA1 + 180);
	this->whl->angle2 = 360 - (target_WA2 + 180);
	this->whl->angle3 = 360 - (target_WA3 + 180);
	this->whl->angle4 = 360 - (target_WA4 + 180);
}
