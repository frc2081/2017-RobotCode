/*
 * CommandBase.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: FIRSTUser
 */

#include "CommandBase.h"


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

bool CommandBase::isDone() {
	return commandComplete;
}
CommandBase::~CommandBase() {
	// TODO Auto-generated destructor stub
}

