/*
 * CommandPause.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: FIRSTUser
 */

#include <autoCommands/CommandPause.h>
#include <stdio.h>

CommandPause::CommandPause(double toWaitSec) {
	// TODO Auto-generated constructor stub

	_initTime = time(NULL);

	_toWaitSec = toWaitSec;

}

commandOutput CommandPause::tick(commandInput input) {
	time_t currentTime = time(NULL);

	time_t elapsed = currentTime - _initTime;


	if (_toWaitSec >= 0 && elapsed >= _toWaitSec)
		setComplete();


	return doNothing();
}

void CommandPause::init(commandInput input) {
}

const char* CommandPause::getCommandName()
{
	return "Pause";
}

CommandPause::~CommandPause() {
	// TODO Auto-generated destructor stub
}

