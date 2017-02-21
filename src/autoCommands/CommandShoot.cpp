/*
 * CommandShoot.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: FIRSTUser
 */

#include <autoCommands/CommandShoot.h>
#include <iostream>

CommandShoot::CommandShoot(double fireTimeSec, double shooterSpd, double shooterAng) {
	// TODO Auto-generated constructor stub

	_toWaitSec = fireTimeSec + 2;
	_shootSpd = shooterSpd;
	_shootAng = shooterAng;
}

void CommandShoot::init(commandInput input) {
	_initTime = time(NULL);
}

const char* CommandShoot::getCommandName()
{
	return "Shoot";
}

commandOutput CommandShoot::tick(commandInput input) {
	time_t currentTime = time(NULL);

	time_t elapsed = currentTime - _initTime;

	printf("Shooting...\n");

	if (elapsed <= 1) return commandOutput(0,0,0, _shootSpd, 0, _shootAng, 0);

	if (_toWaitSec >= 0 && elapsed >= _toWaitSec) {
		setComplete();
		return doNothing();
	}

	//.7 is for short shots, the only time auto shooting will be done
	return commandOutput(0,0,0, _shootSpd, 1, _shootAng, 1);
}

CommandShoot::~CommandShoot() {
	// TODO Auto-generated destructor stub
}

