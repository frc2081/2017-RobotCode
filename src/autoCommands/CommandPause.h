/*
 * CommandPause.h
 *
 *  Created on: Jan 31, 2017
 *      Author: FIRSTUser
 */

#ifndef SRC_AUTOCOMMANDS_COMMANDPAUSE_H_
#define SRC_AUTOCOMMANDS_COMMANDPAUSE_H_
#include "CommandBase.h"
#include <time.h>

class CommandPause : public CommandBase {
public:
	CommandPause(double toWaitSec);
	virtual ~CommandPause();

	commandOutput tick(commandInput input);
	void init(commandInput input);
	const char* getCommandName();

private:
	time_t _initTime;

	double _toWaitSec;
};

#endif /* SRC_AUTOCOMMANDS_COMMANDPAUSE_H_ */
