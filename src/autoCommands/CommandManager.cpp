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
#include <stdio.h>

CommandManager::CommandManager(swervelib *swerveLib, robotTeam team, robotStation station) {
	// TODO Auto-generated constructor stub

	commands = queue<CommandBase*>();
	buildCommands(&commands, team, station);

	_swerveLib = swerveLib;

	currCommand = NULL;

}


commandOutput CommandManager::tick(commandInput input) {


	if (currCommand == NULL || currCommand->isDone()) {
		currCommand = getNextCommand();
		currCommand->init(input);
	}

	return currCommand->tick(input);
}

CommandBase *CommandManager::getNextCommand() {
	CommandBase *commandPop =  commands.front();
	commands.pop();

	if (currCommand != NULL) delete currCommand;

	printf("COMMAND GOTTEN\n");
	return commandPop;
}

void CommandManager::buildCommands(queue<CommandBase*> *queue, robotTeam team, robotStation station) {
	queue->push(new CommandDrive(_swerveLib, 200));
	queue->push(new CommandPause(-1));
}
CommandManager::~CommandManager() {
	// TODO Auto-generated destructor stub
}

