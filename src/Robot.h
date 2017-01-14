/*
 * Robot.h
 *
 *  Created on: Jan 10, 2017
 *      Author: FIRSTUser
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_
#include "cntl.h"
#include "swervelib.h"

cntl *stick1;
cntl *stick2;

swervelib *swerveLib;

ADXRS450_Gyro *gyroCompass;

PIDController *LFPID;
PIDController *LBPID;
PIDController *RFPID;
PIDController *RBPID;
float p, i, d;
const float period = 0.05;

Talon *LFMotDrv;
Talon *LBMotDrv;
Talon *RFMotDrv;
Talon *RBMotDrv;
Talon *LFMotTurn;
Talon *LBMotTurn;
Talon *RFMotTurn;
Talon *RBMotTurn;

AnalogPotentiometer *LFEnc;
AnalogPotentiometer *LBEnc;
AnalogPotentiometer *RFEnc;
AnalogPotentiometer *RBEnc;

enum wheelArray {
	a,
	s
};
#endif /* SRC_ROBOT_H_ */
