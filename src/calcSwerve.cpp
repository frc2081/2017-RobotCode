/*
 * calcSwerve.cpp
 *
 *  Created on: Dec 8, 2015
 *      Author: wentzdr
 */

#include "calcSwerve.h"
#include <iostream>

//a couple of utility functions
double calcSwerve::degToRad(double deg)
{
	return deg*pi/180;
}

double calcSwerve::radToDeg(double rad)
{
	return rad*180/pi;
}

//Standard constructor
calcSwerve::calcSwerve(double chassisXSize, double chassisYSize, double lfWheelAngle, double lrWheelAngle, double rfWheelAngle, double rrWheelAngle)
{
	m_chassisRadius = sqrt(pow(chassisXSize/2, 2) + pow(chassisYSize/2, 2))/12; //calculate the radius of the circle the drive wheels lie on
	m_lfWheelAngle = lfWheelAngle;
	m_lrWheelAngle = lrWheelAngle;
	m_rfWheelAngle = rfWheelAngle;
	m_rrWheelAngle = rrWheelAngle;
}

/*calcSwerve performs all calculations to convert driver input into wheel speeds and angles for a field-oriented Swerve drive
 *
 *Field oriented drive means that when the robot is commanded to move "forward" it will always travel in the same direction,
 *	regardless of what direction the robot is facing at the time. To use this library for robot-oriented drive (a forward command
 *	always moves in the direction of the front of the robot) set the facing input to 0 at all times.
 *
 *The function accepts a motion command in the form of a direction and speed of travel for the center of the robot, a rotational
 *	vector for pivoting the robot around its center axis, and the current facing of the "front" of the robot.
 *
 *Those values are used to calculate the actual angle and velocity each robot wheel must
 *rotate at to produce the commanded robot motion.
 *
 *Inputs:
 *Speed is the absolute magnitude of the desired directional motion of the center of the robot in feet per second
 *Angle is the heading of the desired directional motion of the center of the robot in degrees. 0 degrees representing
 *	the direction of the unit vector(0,1), increasing counterclockwise. Note that this is the opposite of compass headings.
 *rot is the desired rotational motion of the robot about its center axis in radians per second
 *facing is the current facing of the "front" of the robot in compass degrees
 *
 *Outputs:
 *Outputs are stored in the public class variables m_XXAngle and m_XXSpeed, where XX is replaced with the designator for
 * one of the robot's four wheels.
 * Angle is in the same units and frame as the angle input
 * Speed is in units of feet per second
 */
void calcSwerve::calculate(double speed, double angle, double rot, double facing)
{
	//Determine the desired motion of the center of the chassis
	m_centerXVector = speed*cos(degToRad(angle+90));
	m_centerYVector = speed*sin(degToRad(angle+90));

	//Determine the position of each wheel relative to compass heading north
	//This is required because the relative position of each wheel will change as the robot rotates
	m_lfWheelXPos = sin(degToRad(m_lfWheelAngle-facing))*m_chassisRadius;
	m_lrWheelXPos = sin(degToRad(m_lrWheelAngle-facing))*m_chassisRadius;
	m_rfWheelXPos = sin(degToRad(m_rfWheelAngle-facing))*m_chassisRadius;
	m_rrWheelXPos = sin(degToRad(m_rrWheelAngle-facing))*m_chassisRadius;
	m_lfWheelYPos = cos(degToRad(m_lfWheelAngle-facing))*m_chassisRadius;
	m_lrWheelYPos = cos(degToRad(m_lrWheelAngle-facing))*m_chassisRadius;
	m_rfWheelYPos = cos(degToRad(m_rfWheelAngle-facing))*m_chassisRadius;
	m_rrWheelYPos = cos(degToRad(m_rrWheelAngle-facing))*m_chassisRadius;

	//Determine the X and Y vectors for each wheel to fulfill the rotation command
	m_lfWheelRotXVector = rot*m_lfWheelYPos;
	m_lrWheelRotXVector = rot*m_lrWheelYPos;
	m_rfWheelRotXVector = rot*m_rfWheelYPos;
	m_rrWheelRotXVector = rot*m_rrWheelYPos;
	m_lfWheelRotYVector = rot*m_lfWheelXPos;
	m_lrWheelRotYVector = rot*m_lrWheelXPos;
	m_rfWheelRotYVector = rot*m_rfWheelXPos;
	m_rrWheelRotYVector = rot*m_rrWheelXPos;

	//Add the rotation vector for each wheel with the vector for the motion of the chassis center
	//This produces the final vector that each wheel must move through to move the chassis as commanded
	m_lfWheelCombXVector = m_centerXVector - m_lfWheelRotXVector;
	m_lrWheelCombXVector = m_centerXVector - m_lrWheelRotXVector;
	m_rfWheelCombXVector = m_centerXVector - m_rfWheelRotXVector;
	m_rrWheelCombXVector = m_centerXVector - m_rrWheelRotXVector;
	m_lfWheelCombYVector = m_centerYVector + m_lfWheelRotYVector;
	m_lrWheelCombYVector = m_centerYVector + m_lrWheelRotYVector;
	m_rfWheelCombYVector = m_centerYVector + m_rfWheelRotYVector;
	m_rrWheelCombYVector = m_centerYVector + m_rrWheelRotYVector;

	//Calculate the speed each wheel must be driven at to travel the length of it's final vector
	//Just pythagorean theorem
	m_lfSpeed = sqrt(pow(m_lfWheelCombXVector,2)+pow(m_lfWheelCombYVector,2));
	m_lrSpeed = sqrt(pow(m_lrWheelCombXVector,2)+pow(m_lrWheelCombYVector,2));
	m_rfSpeed = sqrt(pow(m_rfWheelCombXVector,2)+pow(m_rfWheelCombYVector,2));
	m_rrSpeed = sqrt(pow(m_rrWheelCombXVector,2)+pow(m_rrWheelCombYVector,2));

	//Determine the angle each wheel must be steered to while being driven
	//The results here are negated and have 90 degrees subtracted to convert them from
	//trigonometry degrees (unit vector (1,0) = 0 degrees) to compass degrees
	//(unit vector (0,1) = 0 degrees)

	//NOTE: IF YOU DO THIS YOURSELF, THE C ATAN2 FUNCTION IS OF THE FORM ATAN2(Y,X)
	//NOT X,Y as might be expected
	m_lfAngle = radToDeg(atan2(m_lfWheelCombYVector, m_lfWheelCombXVector))-90;
	m_lrAngle = radToDeg(atan2(m_lrWheelCombYVector, m_lrWheelCombXVector))-90;
	m_rfAngle = radToDeg(atan2(m_rfWheelCombYVector, m_rfWheelCombXVector))-90;
	m_rrAngle = radToDeg(atan2(m_rrWheelCombYVector, m_rrWheelCombXVector))-90;

	//add 360 degrees to any angle that is negative to ensure they are all positive and
	// between 0 and 360 degrees and therefore ready to be sent to the wheel steering PID controller
	if (m_lfAngle < 0) m_lfAngle += 360;
	if (m_lrAngle < 0) m_lrAngle += 360;
	if (m_rfAngle < 0) m_rfAngle += 360;
	if (m_rrAngle < 0) m_rrAngle += 360;

	//Following is statements to output each step of the calculation for troubleshooting
	//or if you just want to see the calculation steps.
/*
	std::cout<<"lf Wheel Positions: "<<m_lfWheelXPos<<" "<<m_lfWheelYPos<<"\n";
	std::cout<<"lr Wheel Positions: "<<m_lrWheelXPos<<" "<<m_lrWheelYPos<<"\n";
	std::cout<<"rf Wheel Positions: "<<m_rfWheelXPos<<" "<<m_rfWheelYPos<<"\n";
	std::cout<<"rr Wheel Positions: "<<m_rrWheelXPos<<" "<<m_rrWheelYPos<<"\n\n";

	std::cout<<"lf Wheel Rotational Vector: "<<m_lfWheelRotXVector<<" "<<m_lfWheelRotYVector<<"\n";
	std::cout<<"lr Wheel Rotational Vector: "<<m_lrWheelRotXVector<<" "<<m_lrWheelRotYVector<<"\n";
	std::cout<<"rf Wheel Rotational Vector: "<<m_rfWheelRotXVector<<" "<<m_rfWheelRotYVector<<"\n";
	std::cout<<"rr Wheel Rotational Vector: "<<m_rrWheelRotXVector<<" "<<m_rrWheelRotYVector<<"\n\n";

	std::cout<<"lf Wheel Combined Vectors: "<<m_lfWheelCombXVector<<"  "<<m_lfWheelCombYVector<<"\n";
	std::cout<<"lr Wheel Combined Vectors: "<<m_lrWheelCombXVector<<"  "<<m_lrWheelCombYVector<<"\n";
	std::cout<<"rf Wheel Combined Vectors: "<<m_rfWheelCombXVector<<"  "<<m_rfWheelCombYVector<<"\n";
	std::cout<<"rr Wheel Combined Vectors: "<<m_rrWheelCombXVector<<"  "<<m_rrWheelCombYVector<<"\n\n";

	std::cout<<"lf Wheel angle Radians: "<<atan2(m_lfWheelCombYVector, m_lfWheelCombXVector)<<"\n";
	std::cout<<"lr Wheel angle Radians: "<<atan2(m_lrWheelCombYVector, m_lrWheelCombXVector)<<"\n";
	std::cout<<"rf Wheel angle Radians: "<<atan2(m_rfWheelCombYVector, m_rfWheelCombXVector)<<"\n";
	std::cout<<"rr Wheel angle Radians: "<<atan2(m_rrWheelCombYVector, m_rrWheelCombXVector)<<"\n";

*/
}
