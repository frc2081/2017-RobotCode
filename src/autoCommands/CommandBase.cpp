/*
 * CommandBase.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: FIRSTUser
 */

#include <autoCommands/CommandBase.h>


CommandBase::CommandBase() {
	// TODO Auto-generated constructor stub
	commandComplete = false;
}

void CommandBase::setComplete() {
	commandComplete = true;
}

commandOutput CommandBase::doNothing() {
	return commandOutput();
}
CommandBase::~CommandBase() {
	// TODO Auto-generated destructor stub
}

