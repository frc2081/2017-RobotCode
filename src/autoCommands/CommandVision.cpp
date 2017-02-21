/*
 * CommandVision.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: FIRSTUser
 */

#include <autoCommands/CommandVision.h>

CommandVision::CommandVision(swervelib *swerveLib) {
	// TODO Auto-generated constructor stub
	autoMag = 0;
	autoAng = 0;
	autoRot = 0;
	AD = new liftAutoDock();
}

void CommandVision::init(commandInput input) {
}

const char* CommandVision::getCommandName()
{
	return "Vision";
}

commandOutput CommandVision::tick(commandInput input) {

	AD->calcLiftAutoDock(true);
	autoAng = AD->getLADDrvAngCmd();
	autoMag = AD->getLADDrvMagCmd();
	autoRot = AD->getLADDrvRotCmd();

	return commandOutput(autoMag, autoAng, autoRot);
}

CommandVision::~CommandVision() {
	// TODO Auto-generated destructor stub
}

