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
#include "gyroManager.h"
#include "autoCommands/CommandManager.h"

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

extern cntl *cntl1;
extern cntl *cntl2;

extern swervelib *swerveLib;

extern PIDController *LFPID;
extern PIDController *LBPID;
extern PIDController *RFPID;
extern PIDController *RBPID;

extern Encoder *LFEncDrv;
extern Encoder *RFEncDrv;
extern Encoder *LBEncDrv;
extern Encoder *RBEncDrv;

extern float p, i, d;
extern float comAng, comMag;
extern float currentFacing;
extern double currAng1, currAng2, currAng3, currAng4;
const float period = 0.05;

extern VictorSP*ClimbMotDrv;
extern VictorSP *ballLoad;

#endif /* SRC_ROBOT_H_ */
