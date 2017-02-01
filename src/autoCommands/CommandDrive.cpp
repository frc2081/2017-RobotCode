/*
 * CommandDrive.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: FIRSTUser
 */

#include <autoCommands/CommandDrive.h>

CommandDrive::CommandDrive(swervelib *swerveLib, double toTravel) {
	// TODO Auto-generated constructor stub
	_swerveLib = swerveLib;
	_toTravel = toTravel;
}

commandOutput CommandDrive::tick(commandInput input) {

	if (checkDistance(input) >= _toTravel) {
		isDone();
		return doNothing();
	}

	double gyroReading = input.currentGyroReading;

	return commandOutput(.5, gyroReading, 0);

}

void CommandDrive::init(commandInput input) {

	LFWhlDrvEncInit = input.LFWhlDrvEnc;
	RFWhlDrvEncInit = input.RFWhlDrvEnc;
	LBWhlDrvEncInit = input.LBWhlDrvEnc;
	RBWhlDrvEncInit = input.RBWhlDrvEnc;

	LFWhlTurnEncInit = input.LFWhlTurnEnc;
	RFWhlTurnEncInit = input.RFWhlTurnEnc;
	LBWhlTurnEncInit = input.LBWhlTurnEnc;
	RBWhlTurnEncInit = input.RBWhlTurnEnc;

}

double CommandDrive::checkDistance(commandInput input) {

	double LFWhlDrvEnc = fabs(input.LFWhlDrvEnc - LFWhlDrvEncInit);
	double RFWhlDrvEnc = fabs(input.RFWhlDrvEnc - RFWhlDrvEncInit);
	double LBWhlDrvEnc = fabs(input.LBWhlDrvEnc - LBWhlDrvEncInit);
	double RBWhlDrvEnc = fabs(input.RBWhlDrvEnc - RBWhlDrvEncInit);

	double EncAvg = (LFWhlDrvEnc + RFWhlDrvEnc + LBWhlDrvEnc + RBWhlDrvEnc) / 4;

	return EncAvg;

}

CommandDrive::~CommandDrive() {
	// TODO Auto-generated destructor stub
}

