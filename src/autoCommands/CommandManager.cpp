/*
 * CommandManager.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: FIRSTUser
 */

#include <autoCommands/CommandManager.h>
#include "CommandDrive.h"
#include "CommandPause.h"
#include "CommandTurn.h"

CommandManager::CommandManager(swervelib *swerveLib, robotTeam team, robotStation station) {
	// TODO Auto-generated constructor stub

	commands = buildCommands(team, station);

	_swerveLib = swerveLib;
}


commandOutput tick(commandInput input) {

}

vector<CommandBase> CommandManager::buildCommands(robotTeam team, robotStation station) {
	vector<CommandBase> toExecute = vector<CommandBase>();
	toExecute.push_back(CommandDrive(_swerveLib, 2));
	toExecute.push_back(CommandPause(-1));
	return toExecute;
}
CommandManager::~CommandManager() {
	// TODO Auto-generated destructor stub
}

