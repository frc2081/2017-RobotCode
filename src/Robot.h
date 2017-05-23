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
#include "Calibrations.h"
#include "Camera.h"
#include "liftAutoDock.h"
#include "AutoSelector.h"
#include "WPIlib.h"
#include "CANTalon.h"

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
extern CANTalon *ballShooterMot;

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
bool shooterToggle;
double shooterAngle;
int shooterSelection;
extern double shooterAimLocation;
const float period = 0.05;

Relay *gearLoader;

bool autoDockCmd;

AnalogInput *autoEnable;
AutoSelector *autoFieldPosition;
AutoSelector *autoAction;

liftAutoDock *AD;

extern commandInput autoInput;
extern commandOutput autoOutput;

double lfWhlAngCalOffset = 0;
double rfWhlAngCalOffset = 0;
double lbWhlAngCalOffset = 0;
double rbWhlAngCalOffset = 0;
DigitalInput *WhlAngCalButton;
Preferences *prefs;

#endif /* SRC_ROBOT_H_ */
