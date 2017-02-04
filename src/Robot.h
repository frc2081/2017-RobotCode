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
#include "autoCommands\CommandManager.h"
#include "autoCommands\CommandIO.h"

extern VictorSP *LFMotDrv;
extern VictorSP *LBMotDrv;
extern VictorSP *RFMotDrv;
extern VictorSP *RBMotDrv;
extern VictorSP *LFMotTurn;
extern VictorSP *LBMotTurn;
extern VictorSP *RFMotTurn;
extern VictorSP *RBMotTurn;
extern VictorSP *ballFeederMot;
extern VictorSP *ClimbMotDrv;
extern VictorSP *ballLoad;
extern VictorSP *ballShooterMot;

extern Servo *shooterAimServo;

extern AnalogPotentiometer *LFEncTurn;
extern AnalogPotentiometer *LBEncTurn;
extern AnalogPotentiometer *RFEncTurn;
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
extern double feederSpeed;
extern bool runShooter;
extern double shooterAimLocation;
const float period = 0.05;

extern commandInput autoInput;
extern commandOutput autoOutput;

#endif /* SRC_ROBOT_H_ */
