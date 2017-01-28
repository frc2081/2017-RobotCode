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
	double target_WSRF, target_WSLF, target_WSLB, target_WSRB, MAX_WS;
	double target_WARF, target_WALF, target_WALB, target_WARB;
	double A, B, C, D, radius, _width, _length;
	double currAngRF, currAngLF, currAngLB, currAngRB;
	double centerVecX, centerVecY;

	int a [11] = { 0, 0, 45, 90, 135, 180, 225, 280, 325, 360, 360 };
public:
    swervelib(double wheelbase, double trackwidth);

    virtual ~swervelib() = default;

    wheel *whl;

    void calcWheelVect(double mag, double ang, double rotation);
};
#endif //SWERVELIB_H_
