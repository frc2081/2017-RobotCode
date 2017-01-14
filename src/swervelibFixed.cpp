#include <iostream>
#include "swervelibFixed.h"

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

swervelib::swervelib(double wheelbase, double trackwidth) {
	target_WS1 = 0;
	target_WS2 = 0;
	target_WS3 = 0;
	target_WS4 = 0;
	MAX_WS = 0;

	LFWhlAng = 0;
	LBWhlAng = 0;
	RBWhlAng = 0;
	RFWhlAng = 0;

	LFWhlSpeed = 0;
	LBWhlSpeed = 0;
	RBWhlSpeed = 0;
	RFWhlSpeed = 0;
	R = sqrt(pow(wheelbase/2, 2) + pow(trackwidth/2, 2))/12;
	_wheelbase = wheelbase;

	whl = new wheel();
}

void swervelib::calcWheelVect(double x,
						      double y,
						      double rudder,
							  double facing) {

	if (fabs(x) < 0.2) { x = 0.0; }
	if (fabs(y) < 0.2) { y = 0.0; }
	if (fabs(rudder) < 0.2) { rudder = 0.0; }

	double speed = sqrt(pow(x, 2) + pow(y, 2)) * 11.5;
	double angle = atan2(y, x) * 180/PI;
	double rot = rudder * 11.5;

	double centerXVec = speed*cos(degrees_to_radians(angle+90));
	double centerYVec = speed*sin(degrees_to_radians(angle+90));

	double LFXWhlPos = sin(degrees_to_radians(LFWhlAng-facing));
	double RFXWhlPos = sin(degrees_to_radians(RFWhlAng-facing));
	double LBXWhlPos = sin(degrees_to_radians(LBWhlAng-facing));
	double RBXWhlPos = sin(degrees_to_radians(RBWhlAng-facing));
	double LFYWhlPos = cos(degrees_to_radians(LFWhlAng-facing));
	double RFYWhlPos = cos(degrees_to_radians(RFWhlAng-facing));
	double LBYWhlPos = cos(degrees_to_radians(LBWhlAng-facing));
	double RBYWhlPos = cos(degrees_to_radians(RBWhlAng-facing));

	double LFXRot = LFYWhlPos*rot;
	double LBXRot = LBYWhlPos*rot;
	double RFXRot = RFYWhlPos*rot;
	double RBXRot = RBYWhlPos*rot;
	double LFYRot = LFXWhlPos*rot;
	double LBYRot = LBXWhlPos*rot;
	double RFYRot = RFXWhlPos*rot;
	double RBYRot = RBXWhlPos*rot;

	double LFMotionVecX = centerXVec - LFXRot;
	double LBMotionVecX = centerXVec - LBXRot;
	double RFMotionVecX = centerXVec - RFXRot;
	double RBMotionVecX = centerXVec - RBXRot;
	double LFMotionVecY = centerYVec - LFYRot;
	double LBMotionVecY = centerYVec - LBYRot;
	double RFMotionVecY = centerYVec - RFYRot;
	double RBMotionVecY = centerYVec - RBYRot;
	target_WS1 = sqrt(pow(LFMotionVecX, 2) + pow(LFMotionVecY, 2)); //1
	target_WS2 = sqrt(pow(LBMotionVecX, 2) + pow(LBMotionVecY, 2)); //2
	target_WS3 = sqrt(pow(RFMotionVecX, 2) + pow(RFMotionVecY, 2)); //3
	target_WS4 = sqrt(pow(RBMotionVecX, 2) + pow(RBMotionVecY, 2)); //4

	/*
	* Gives the angle in radians between the positive X axis and the point given
	* by the coordinates.  The result is then converted to degrees.
	*/
	this->whl->angle1 = radians_to_degrees(atan2(RFMotionVecY, RFMotionVecX));
	this->whl->angle2 = radians_to_degrees(atan2(LFMotionVecY, LFMotionVecX));
	this->whl->angle3 = radians_to_degrees(atan2(LBMotionVecY, LBMotionVecX));
	this->whl->angle4 = radians_to_degrees(atan2(RBMotionVecY, RBMotionVecX));

	//if (this->whl->angle1 < 0) {this->whl->angle1 += 360;}
	//if (this->whl->angle2 < 0) {this->whl->angle2 += 360;}
	//if (this->whl->angle3 < 0) {this->whl->angle3 += 360;}
	//if (this->whl->angle4 < 0) {this->whl->angle4 += 360;}

	this->whl->speed1 = target_WS1;
	this->whl->speed2 = target_WS2;
	this->whl->speed3 = target_WS3;
	this->whl->speed4 = target_WS4;
}

