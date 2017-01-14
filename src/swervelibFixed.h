/*
 * swervelibFixed.h
 *
 *  Created on: Jan 11, 2017
 *      Author: FIRSTUser
 */

#ifndef SRC_SWERVELIBFIXED_H_
#define SRC_SWERVELIBFIXED_H_
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
	double LFWhlAng, LBWhlAng, RFWhlAng, RBWhlAng;
	double LFWhlSpeed, LBWhlSpeed, RFWhlSpeed, RBWhlSpeed;
	double R, _wheelbase;

public:
    swervelib(double wheelbase, double trackwidth);

    virtual ~swervelib() = default;

    wheel *whl;

    void calcWheelVect(double x, double y, double rudder, double facing);
};



#endif /* SRC_SWERVELIBFIXED_H_ */
