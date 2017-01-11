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

	PIDController *LFPID;
	PIDController *LBPID;
	PIDController *RFPID;
	PIDController *RBPID;
	float p, i, d;
	const float period = 0.05;

	VictorSP *LFMotDrv;
	VictorSP *LBMotDrv;
	VictorSP *RFMotDrv;
	VictorSP *RBMotDrv;
	VictorSP *LFMotTurn;
	VictorSP *LBMotTurn;
	VictorSP *RFMotTurn;
	VictorSP *RBMotTurn;

	Encoder *LFEnc;
	Encoder *LBEnc;
	Encoder *RFEnc;
	Encoder *RBEnc;

#endif /* SRC_ROBOT_H_ */
