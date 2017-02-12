/*
 * Calibrations.h
 *
 *  Created on: Feb 11, 2017
 *      Author: blzzrd
 */

#ifndef SRC_CALIBRATIONS_H_
#define SRC_CALIBRATIONS_H_

//SHOOTER cals
double shooterAimIncrement = .1;

double shooterAngNearShot = .2;
double shooterAngFarShot = .8;

double shooterSpdNearShot = .7;
double shooterSpdFarShot = 1;

//FEEDER cals
double feederSpeedHigh = 1;

//INTAKE cals
double fuelIntakeSpeedForward = 1;
double fuelIntakeSpeedReverse = 1;

//TURN MOTOR PID cals
double turnMotorP = .25;
double turnMotorI = 0;
double turnMotorD = 0;

//DRIVE MOTOR cals
double drvWhlDistPerEncCnt = .1904545454545;

#endif /* SRC_CALIBRATIONS_H_ */
