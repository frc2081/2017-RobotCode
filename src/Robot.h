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

extern Talon *LFMotDrv;
extern Talon *LBMotDrv;
extern Talon *RFMotDrv;
extern Talon *RBMotDrv;
extern Talon *LFMotTurn;
extern Talon *LBMotTurn;
extern Talon *RFMotTurn;
extern Talon *RBMotTurn;

extern AnalogPotentiometer *LFEnc;
extern AnalogPotentiometer *LBEnc;
extern AnalogPotentiometer *RFEnc;
extern AnalogPotentiometer *RBEnc;

extern cntl *stick1;
extern cntl *stick2;

extern swervelib *swerveLib;

extern ADXRS450_Gyro *gyroCompass;

extern PIDController *LFPID;
extern PIDController *LBPID;
extern PIDController *RFPID;
extern PIDController *RBPID;

extern Encoder *LFEncDrv;
extern Encoder *RFEncDrv;
extern Encoder *LBEncDrv;
extern Encoder *RBEncDrv;

extern float p, i, d;
extern bool charlesMode;
bool bStartHld;
bool bStart;
const float period = 0.05;


#endif /* SRC_ROBOT_H_ */
