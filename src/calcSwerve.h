/*
 * calcSwerve.h
 *
 *  Created on: Dec 8, 2015
 *      Author: wentzdr
 */

#ifndef CALCSWERVE_H_
#define CALCSWERVE_H_

#define pi 3.14159265

#include <math.h>

class calcSwerve
{
public:
	calcSwerve(double chassisXSize, double chassisYSize, double lfWheelAngle, double lrWheelAngle, double rfWheelAngle, double rrWheelAngle);

	double m_lfAngle, m_lrAngle, m_rfAngle, m_rrAngle;
	double m_lfSpeed, m_lrSpeed, m_rfSpeed, m_rrSpeed;

	void calculate(double speed, double angle, double rot, double facing);

private:

	double m_chassisRadius;

	double m_lfWheelAngle, m_lrWheelAngle, m_rfWheelAngle, m_rrWheelAngle;

	double m_lfWheelXPos, m_lrWheelXPos, m_rfWheelXPos, m_rrWheelXPos;
	double m_lfWheelYPos, m_lrWheelYPos, m_rfWheelYPos, m_rrWheelYPos;

	double m_centerXVector, m_centerYVector;

	double m_lfWheelRotXVector, m_lrWheelRotXVector, m_rfWheelRotXVector, m_rrWheelRotXVector;
	double m_lfWheelRotYVector, m_lrWheelRotYVector, m_rfWheelRotYVector, m_rrWheelRotYVector;

	double m_lfWheelDirXVector, m_lrWheelDirXVector, m_rfWheelDirXVector, m_rrWheelDirXVector;
	double m_lfWheelDirYVector, m_lrWheelDirYVector, m_rfWheelDirYVector, m_rrWheelDirYVector;

	double m_lfWheelCombXVector, m_lrWheelCombXVector, m_rfWheelCombXVector, m_rrWheelCombXVector;
	double m_lfWheelCombYVector, m_lrWheelCombYVector, m_rfWheelCombYVector, m_rrWheelCombYVector;

	double radToDeg(double rad);
	double degToRad(double deg);
};

#endif /* CALCSWERVE_H_ */
