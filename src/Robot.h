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
#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "Robot.h"
#include "Calibrations.h"
#include "Camera.h"

extern VictorSP *LFMotDrv;
extern VictorSP *LBMotDrv;
extern VictorSP *RFMotDrv;
extern VictorSP *RBMotDrv;
extern VictorSP *LFMotTurn;
extern VictorSP *LBMotTurn;
extern VictorSP *RFMotTurn;
extern VictorSP *RBMotTurn;
extern VictorSP *ballFeederMot;
extern VictorSP *ClimbMotDrv1;
extern VictorSP *ClimbMotDrv2;
extern VictorSP *ClimbMotDrv3;
extern VictorSP *ballLoad;
extern VictorSP *ballShooterMot;

extern Servo *shooterAimServo;

extern AnalogPotentiometer *LFEncTurn;
extern AnalogPotentiometer *LBEncTurn;
extern AnalogPotentiometer *RFEncTurn;
extern AnalogPotentiometer *RBEncTurn;

extern cntl *cntl1;
extern cntl *cntl2;

extern swervelib *swerveLib;

extern CameraServer *alignCam;

extern PIDController *LFPID;
extern PIDController *LBPID;
extern PIDController *RFPID;
extern PIDController *RBPID;

extern Encoder *LFEncDrv;
extern Encoder *RFEncDrv;
extern Encoder *LBEncDrv;
extern Encoder *RBEncDrv;
extern Encoder *shooterEnc;

extern float p, i, d;
extern float comAng, comMag;
extern float currentFacing;
extern double currAng1, currAng2, currAng3, currAng4;
extern double feederSpeed;
extern bool runShooter;
extern double shooterAimLocation;
const float period = 0.05;

extern double liftCenterDistance;
enum autoGearStates {
	//Driver has not yet engaged the autolock
	DO_NOTHING,
	//Make sure the target is in the camera's field of vision
	TARGET_AQUIRED,
	//Make the robot square against the desired target
	TURN_TO_SQUARE,
	//Line the robot up to drive straight into the spring
	HORIZONTAL_LINEUP,
	//Drive to the proper distance to unload the spring
	DRIVE_TO_SPRING,
	//Wait a few seconds, then let normal drive controls work again
	DONE
};
extern autoGearStates autoGearStateMachine;

//extern CAMERAFEEDS *cameras;

extern commandInput autoInput;
extern commandOutput autoOutput;

#endif /* SRC_ROBOT_H_ */
