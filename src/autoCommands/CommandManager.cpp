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

	currCommand = NULL;

	currCommand = getNextCommand();
}


commandOutput CommandManager::tick(commandInput input) {

	if (currCommand->isDone()) currCommand = getNextCommand();

	return currCommand->tick(input);
}

CommandBase *CommandManager::getNextCommand() {
	CommandBase *commandPop =  commands.front();
	commands.pop();

	if (currCommand != NULL) delete currCommand;

	return commandPop;
}

queue<CommandBase*> CommandManager::buildCommands(robotTeam team, robotStation station) {
	queue<CommandBase*> toExecute = queue<CommandBase*>();
	toExecute.push(new CommandDrive(_swerveLib, 2));
	toExecute.push(new CommandPause(-1));
	return toExecute;
}
CommandManager::~CommandManager() {
	// TODO Auto-generated destructor stub
}

