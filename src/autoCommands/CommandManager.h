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
#include <vector>

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


private:
	swervelib *_swerveLib;

	vector<CommandBase> commands;

	vector<CommandBase> buildCommands(robotTeam, robotStation);
};

#endif /* SRC_AUTOCOMMANDS_COMMANDMANAGER_H_ */
