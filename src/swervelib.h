/* Copyright 2016 Charles Young */

#ifndef SWERVELIB_H_
#define SWERVELIB_H_

#include <cmath>

const double PI = 3.14159265;

class wheel {
public:
	// front right, front left, rear left, rear right
	double speed1, speed2, speed3, speed4;
	// front right, front left, rear left, rear right
	double angle1, angle2, angle3, angle4;
};

class swervelib {
private:
	double target_WS1, target_WS2, target_WS3, target_WS4, MAX_WS;
	double target_WA1, target_WA2, target_WA3, target_WA4;
	double A, B, C, D, R, _width, _length;
	double currAng1, currAng2, currAng3, currAng4;
	double centerVecX, centerVecY;

	int a [11] = { 0, 0, 45, 90, 135, 180, 225, 280, 325, 360, 360 };
public:
    swervelib(double wheelbase, double trackwidth);

    virtual ~swervelib() = default;

    wheel *whl;

    void calcWheelVect(double mag, double ang, double rotation);
};
#endif //SWERVELIB_H_
