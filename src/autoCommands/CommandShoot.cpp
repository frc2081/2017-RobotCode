/*
 * CommandShoot.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: FIRSTUser
 */

#include <autoCommands/CommandShoot.h>

CommandShoot::CommandShoot(double fireTimeSec) {
	// TODO Auto-generated constructor stub

	_toWaitSec = fireTimeSec + 2;

}

void CommandShoot::init(commandInput input) {
	_initTime = time(NULL);
}
commandOutput CommandShoot::tick(commandInput input) {
	time_t currentTime = time(NULL);

	time_t elapsed = currentTime - _initTime;

	if (elapsed <= 1) return commandOutput(.7, 0);

	if (_toWaitSec >= 0 && elapsed >= _toWaitSec) {
		setComplete();
		return doNothing();
	}

	//.7 is for short shots, the only time auto shooting will be done
	return commandOutput(.7, 1);
}

CommandShoot::~CommandShoot() {
	// TODO Auto-generated destructor stub
}

