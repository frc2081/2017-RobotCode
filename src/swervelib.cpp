/* Copyright 2016 Charles Young */

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

enum wheelArray {
	a,
	s
};

swervelib::swervelib(double _length, double _width) {
	W1[a] = 0;
	W2[a] = 0;
	W3[a] = 0;
	W4[a] = 0;
	W1[s] = 0;
	W2[s] = 0;
	W3[s] = 0;
	W4[s] = 0;
	MAX_WS = 0;

	A = 0;
	B = 0;
	C = 0;
	D = 0;
	R = sqrt(pow(width, 2) + pow(length, 2));
	width = _width;
	length = _length;
}

void swervelib::calcWheelVect(double x,
						 	  double y,
							  double rudder) {

	A = x - rudder * (length/2);
	B = x + rudder * (length/2);
	C = y - rudder * (width/2);
	D = y + rudder * (width/2);

	W1[s] = sqrt(pow(B, 2) + pow(C, 2));
	W2[s] = sqrt(pow(B, 2) + pow(D, 2));
	W3[s] = sqrt(pow(A, 2) + pow(D, 2));
	W4[s] = sqrt(pow(A, 2) + pow(C, 2));
	MAX_WS = std::max({W1[s], W2[s], W3[s], W4[s]});

	if(MAX_WS > 1) {
		W1[s] /= MAX_WS;
		W2[s] /= MAX_WS;
		W3[s] /= MAX_WS;
		W4[s] /= MAX_WS;
	}

	W1[a] = (atan2(B, C) * 180 ) / PI;
	W2[a] = (atan2(B, D) * 180 ) / PI;
	W3[a] = (atan2(A, D) * 180 ) / PI;
	W4[a] = (atan2(A, C) * 180 ) / PI;

}
