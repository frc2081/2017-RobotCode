/* Copyright 2016 Charles Young */

#ifndef SWERVELIB_H_
#define SWERVELIB_H_

#include <cmath>

const double PI = 3.14159265;

class wheel {
public:
	// front right, front left, rear left, rear right
	double speedRF, speedLF, speedLB, speedRB;
	// front right, front left, rear left, rear right
	double angleRF, angleLF, angleLB, angleRB;
};

class swervelib {
private:
	//Wanted speeds for the wheels
	double _targetWhlSpeed_RF, _targetWhlSpeed_LF, _targetWhlSpeed_LB, _targetWhlSpeed_RB, _MaxWhlSpeed;
	//Wanted angles for the wheels
	double _targetWhlAng_RF, _targetWhlAng_LF, _targetWhlAng_LB, _targetWhlAng_RB;
	//Vectors for the wheels' motion, and dimensions of robot. Width is wheelbase, length is trachwidth, radius is one corrner to center
	double _A, _B, _C, _D, _radius, _width, _length;
	//Last commanded angle for wheels
	double _currAngRF, _currAngLF, _currAngLB, _currAngRB;
	//Motion the center of the robot wants to take
	double _centerVecX, _centerVecY;
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
public:
    swervelib(double wheelbase, double trackwidth);

    virtual ~swervelib() = default;

    wheel *whl;

    void calcWheelVect(double mag, double ang, double rotation);
};
#endif //SWERVELIB_H_
