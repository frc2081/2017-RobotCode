/*
 * Calibrations.h
 *
 *  Created on: Feb 11, 2017
 *      Author: blzzrd
 */

#ifndef SRC_CALIBRATIONS_H_
#define SRC_CALIBRATIONS_H_

//SHOOTER cals
double shooterSpdP = 0.001;
double shooterSpdI = 0.01;
double shooterSpdD = 0;
double shooterMaxRevPerSec = 88.5;
double shooterEncCntPerRev = .05;

double shooterAimIncrement = .1;
double shooterAngNearShot = .2;
double shooterAngFarShot = .8;
double shooterPwrNearShot = .7; //POWER is for directly commanding the shooter motor
double shooterPwrFarShot = 1;
double shooterSpdNearShot = 50; //SPEED is for PID control of shooter speed
double shooterSpdFarShot = shooterMaxRevPerSec;


//FEEDER cals
double feederSpeedHigh = 1;

//INTAKE cals
double fuelIntakeSpeedForward = 1;
double fuelIntakeSpeedReverse = -1;

//TURN MOTOR PID cals
double turnMotorP = .025;
double turnMotorI = 0;
double turnMotorD = 0;

//DRIVE MOTOR cals
double drvWhlDistPerEncCnt = .1904545454545;

#endif /* SRC_CALIBRATIONS_H_ */
