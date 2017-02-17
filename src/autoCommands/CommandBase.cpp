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

bool CommandBase::compareDouble(double x, double y) {
	double difference = fabs(x - y);
	return difference <= 1;
}
CommandBase::~CommandBase() {
	// TODO Auto-generated destructor stub
}

