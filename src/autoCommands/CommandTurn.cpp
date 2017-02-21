/*
 * CommandTurn.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: FIRSTUser
 */

#include <autoCommands/CommandTurn.h>
#include <iostream>

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

	double delta = gyroReadingInit - _finalRot;
	if(abs(delta) > 180){
		if(delta > 0) _turnDirection = 1;
		else _turnDirection = -1;
	} else {
		if(delta > 0) _turnDirection = -1;
		else _turnDirection = 1;
	}
}

const char* CommandTurn::getCommandName()
{
	return "Turn";
}

commandOutput CommandTurn::tick(commandInput input) {
	printf("TURNING\n");
	gyroReading =  input.currentGyroReading - gyroReadingInit;
	if (gyroReading >= 360) gyroReading = ((int)gyroReading % 360);
	if (gyroReading <= 0) gyroReading = 360 + gyroReading;
	if (gyroReading >= _finalRot - _targetTolerance && gyroReading <= _finalRot + _targetTolerance) {
		setComplete();
		printf("TURN COMPLETE\n");
		return doNothing();
	}

	printf("Target: %f Gyro %f\n", _finalRot, gyroReading );

	return commandOutput(0, 0, _turnRate * _turnDirection);
}

CommandTurn::~CommandTurn() {
	// TODO Auto-generated destructor stub
}

