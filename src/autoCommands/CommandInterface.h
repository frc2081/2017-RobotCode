/*
 * CommandInterface.h
 *
 *  Created on: Jan 31, 2017
 *      Author: FIRSTUser
 */

#ifndef SRC_AUTOCOMMANDS_COMMANDINTERFACE_H_
#define SRC_AUTOCOMMANDS_COMMANDINTERFACE_H_
#include "CommandIO.h"

class CommandInterface {
public:
	virtual commandOutput tick(commandInput input) = 0;
	virtual void init(commandInput input) = 0;
	virtual bool isDone() = 0;
	virtual const char* getCommandName() = 0;
};


#endif /* SRC_AUTOCOMMANDS_COMMANDINTERFACE_H_ */
