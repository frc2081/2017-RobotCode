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
#include "CommandBase.h"
#include <queue>

using namespace std;

enum robotTeam {
	BLUE,
	RED
};

enum robotStation {
	ONE,
	TWO,
	THREE
};
class CommandManager {
public:
	CommandManager(swervelib *swerveLib, robotTeam, robotStation);
	virtual ~CommandManager();
	commandOutput tick(commandInput input);

protected:
	CommandBase *getNextCommand();
private:
	swervelib *_swerveLib;

	queue<CommandBase*> commands;

	void buildCommands(queue<CommandBase*> *queue,robotTeam, robotStation);

	CommandBase *currCommand;
};

#endif /* SRC_AUTOCOMMANDS_COMMANDMANAGER_H_ */
