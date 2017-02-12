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
	_targetWhlSpeed_RF = 0;
	_targetWhlSpeed_LF = 0;
	_targetWhlSpeed_LB = 0;
	_targetWhlSpeed_RB = 0;
	_targetWhlAng_RF = 0;
	_targetWhlAng_LF = 0;
	_targetWhlAng_LB = 0;
	_targetWhlAng_RB = 0;
	_currAngRF = 0;
	_currAngLF = 0;
	_currAngLB = 0;
	_currAngRB = 0;
	_MaxWhlSpeed = 0;

	_A = 0;
	_B = 0;
	_C = 0;
	_D = 0;
	//Radius of robot - corner to center
	_radius = sqrt(pow(width, 2) + pow(length, 2));
	_width = width;
	_length = length;

	whl = new wheel();
}

void swervelib::calcWheelVect(double mag,
						 double ang,
						 double rotation) {

	//Constructs a X and Y vector from the given magnitude and angle
	_centerVecX = mag * cos(degrees_to_radians(ang+90));
	_centerVecY = mag * sin(degrees_to_radians(ang+90));

	//Last commanded wheel angles
	_currAngRF = this->whl->angleRF;
	_currAngLF = this->whl->angleLF;
	_currAngLB = this->whl->angleLB;
	_currAngRB = this->whl->angleRB;

	//Calculate the wheel motion vectors.
	/*
	 * Wheel 1
	 * 	X = B
	 * 	Y = C
	 * Wheel 2
	 *	X = B
	 *	Y = D
	 * Wheel 3
	 * 	X = A
	 * 	Y = D
	 * Wheel 4
	 * 	X = A
	 * 	Y = C
*/
	_A = _centerVecX - rotation * (_length/_radius);
	_B = _centerVecX + rotation * (_length/_radius);
	_C = _centerVecY - rotation * (_width/_radius);
	_D = _centerVecY + rotation * (_width/_radius);

	//Calculate the wheel speeds. Only Pythagroean Therom
	_targetWhlSpeed_RF = sqrt(pow(_B, 2) + pow(_C, 2));
	_targetWhlSpeed_LF = sqrt(pow(_B, 2) + pow(_D, 2));
	_targetWhlSpeed_LB = sqrt(pow(_A, 2) + pow(_D, 2));
	_targetWhlSpeed_RB = sqrt(pow(_A, 2) + pow(_C, 2));
	_MaxWhlSpeed = std::max({_targetWhlSpeed_RF, _targetWhlSpeed_LF, _targetWhlSpeed_LB, _targetWhlSpeed_RB});

	//Reducing any speed that is over 1, the max a motor can be commanded, to 1
	if (_MaxWhlSpeed > 1) {
		_targetWhlSpeed_RF /= _MaxWhlSpeed;
		_targetWhlSpeed_LF /= _MaxWhlSpeed;
		_targetWhlSpeed_LB /= _MaxWhlSpeed;
		_targetWhlSpeed_RB /= _MaxWhlSpeed;
	}
	//Calculating wanted angle of each wheel by taking an arc tangent of the wheel's vectors
	_targetWhlAng_RF = radians_to_degrees(atan2(_B, _C));
	_targetWhlAng_LF = radians_to_degrees(atan2(_B, _D));
	_targetWhlAng_LB = radians_to_degrees(atan2(_A, _D));
	_targetWhlAng_RB = radians_to_degrees(atan2(_A, _C));


	//Set the wheel speeds and angles to be accessed outside of the class
	this->whl->speedRF = _targetWhlSpeed_RF;
	this->whl->speedLF = _targetWhlSpeed_LF;
	this->whl->speedLB = _targetWhlSpeed_LB;
	this->whl->speedRB = _targetWhlSpeed_RB;

	this->whl->angleRF = 360 - (_targetWhlAng_RF + 180);
	this->whl->angleLF = 360 - (_targetWhlAng_LF + 180);
	this->whl->angleLB = 360 - (_targetWhlAng_LB + 180);
	this->whl->angleRB = 360 - (_targetWhlAng_RB + 180);
}
