/*
 * CommandTurn.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: FIRSTUser
 */

#include <autoCommands/CommandTurn.h>

CommandTurn::CommandTurn(swervelib *swerveLib, double toRotate) {
	// TODO Auto-generated constructor stub

	_toRotate = toRotate;
	currentState = wheelTurn;
}

void CommandTurn::init (commandInput input) {

	gyroReadingInit = input.currentGyroReading;

	double comboAng = gyroReadingInit + ((int)_toRotate % 360);
	if (comboAng > 360) {
		comboAng -= 360;
	} else if (comboAng < 0) {
		comboAng += 360;
	}
	_finalRot = comboAng;

}

commandOutput CommandTurn::tick(commandInput input) {
	gyroReading =  input.currentGyroReading - gyroReadingInit;


	if (input.currentGyroReading == _finalRot) {
		isDone();
		return doNothing();
	}

	return commandOutput(0, _finalRot, .5);
}

CommandTurn::~CommandTurn() {
	// TODO Auto-generated destructor stub
}

