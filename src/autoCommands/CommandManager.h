/*
 * CommandManager.h
 *
 *  Created on: Jan 30, 2017
 *      Author: FIRSTUser
 */

#ifndef SRC_AUTOCOMMANDS_COMMANDMANAGER_H_
#define SRC_AUTOCOMMANDS_COMMANDMANAGER_H_

#include "..\swervelib.h"
#include "CommandIO.h"
class CommandManager {
public:
	CommandManager(swervelib *swerveLib);
	virtual ~CommandManager();
	commandOutput tick(commandInput input);

private:
	swervelib *swerveLib;
};

#endif /* SRC_AUTOCOMMANDS_COMMANDMANAGER_H_ */
