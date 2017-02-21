/*
 * CommandDrive.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: FIRSTUser
 */

#include <autoCommands/CommandDrive.h>
#include <stdio.h>

CommandDrive::CommandDrive(swervelib *swerveLib, double toTravel, double direction) {
	// TODO Auto-generated constructor stub
	_swerveLib = swerveLib;
	_toTravel = toTravel;
	_direction = direction;
}

commandOutput CommandDrive::tick(commandInput input) {
	printf("DRIVING\n");

	if (checkDistance(input) >= _toTravel) {
		setComplete();
		printf("DRIVE COMPLETE\n");
		return doNothing();
	}

	return commandOutput(.5, _direction, 0);
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

const char* CommandDrive::getCommandName()
{
	return "Drive";
}

double CommandDrive::checkDistance(commandInput input) {

	double LFWhlDrvEnc = fabs(input.LFWhlDrvEnc - LFWhlDrvEncInit);
	double RFWhlDrvEnc = fabs(input.RFWhlDrvEnc - RFWhlDrvEncInit);
	double LBWhlDrvEnc = fabs(input.LBWhlDrvEnc - LBWhlDrvEncInit);
	double RBWhlDrvEnc = fabs(input.RBWhlDrvEnc - RBWhlDrvEncInit);

	double EncAvg = (LFWhlDrvEnc + RFWhlDrvEnc + LBWhlDrvEnc + RBWhlDrvEnc) / 4;

	printf("ENcAvg: %f", EncAvg);

	return EncAvg;

}

CommandDrive::~CommandDrive() {
	// TODO Auto-generated destructor stub
}

