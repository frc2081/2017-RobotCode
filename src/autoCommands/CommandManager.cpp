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
#include "CommandVision.h"
#include "CommandShoot.h"
#include <stdio.h>

CommandManager::CommandManager(swervelib *swerveLib, robotTeam team, robotStation station, robotAction action) {
	// TODO Auto-generated constructor stub

	commands = queue<CommandBase*>();
	buildCommands(&commands, team, station, action);

	_swerveLib = swerveLib;

	currCommand = NULL;

}

commandOutput CommandManager::tick(commandInput input) {


	if (currCommand == NULL || currCommand->isDone()) {
		printf("COMMAND COMPLETE. GETTING NEXT COMMAND\n");
		currCommand = getNextCommand();
		currCommand->init(input);
	}

	return currCommand->tick(input);
}

CommandBase *CommandManager::getNextCommand() {
	CommandBase *commandPop =  commands.front();
	commands.pop();

	//Need to add NULL protection here in case no command was added to the queue prior to the pop
	if (currCommand != NULL) delete currCommand;

	printf("COMMAND GOTTEN\n");
	return commandPop;
}

void CommandManager::buildCommands(queue<CommandBase*> *queue, robotTeam team, robotStation station, robotAction action) {
	switch(action) {
	case CROSS_MIDLINE:
		crossMidline(queue, team, station);
		break;
	case GEAR_ONLY:
		gearOnly(queue, team, station);
		break;
	case SHOOT_ONLY:
		shootOnly(queue, team, station);
		break;
	case GEAR_AND_SHOOT:
		gearAndShoot(queue, team, station);
		break;
	case SHOOT_ONLY_BIN:
		shootOnlyBin(queue, team, station);
		break;
	default:
		break;
	}
}

//ALL VALUES IN COMMANDS ARE PLACEHOLDERS FOR ACTUAL VALUES
void CommandManager::crossMidline(queue<CommandBase*> *queue,robotTeam team, robotStation station) {
	if (station == TWO) {
		queue->push(new CommandDrive(_swerveLib, 40, 0));
		//queue->push(new CommandTurn(_swerveLib, 90));
		queue->push(new CommandDrive(_swerveLib, 84, 270));
		//queue->push(new CommandTurn(_swerveLib, 270));
		queue->push(new CommandDrive(_swerveLib, 100, 0));
	} else { queue->push(new CommandPause(1)); }
}

void CommandManager::gearOnly(queue<CommandBase*> *queue,robotTeam team, robotStation station) {
	if (station == TWO) {
		//drive forward 50 inches
		//queue->push(new CommandDrive(_swerveLib, 50, 90));
		queue->push(new CommandVision(_swerveLib));
		return;
	}
	if ((station == ONE && team == RED) || (station == THREE && team == BLUE)) {
		//forward 75 inches
		//turn 45 degrees right
			queue->push(new CommandDrive(_swerveLib, 45, 0));
			queue->push(new CommandTurn (_swerveLib, 45));
			queue->push(new CommandVision(_swerveLib));
		} else {
			queue->push(new CommandDrive(_swerveLib, 20, 0));
			queue->push(new CommandTurn (_swerveLib, 330));
			queue->push(new CommandVision(_swerveLib));
		}

}

void CommandManager::shootOnly(queue<CommandBase*> *queue,robotTeam team, robotStation station) {


	queue->push(new CommandShoot(10, configShooterSpd(station), configShooterAng(station)));

}

void CommandManager::gearAndShoot(queue<CommandBase*> *queue,robotTeam team, robotStation station) {

		queue->push(new CommandShoot(10, configShooterSpd(station), configShooterAng(station)));
	if (team == BLUE) {
		queue->push(new CommandDrive(_swerveLib, 20, 0));
		queue->push(new CommandTurn(_swerveLib, 130));
		queue->push(new CommandVision(_swerveLib));
	} else {
		queue->push(new CommandDrive(_swerveLib, 20, 0));
		queue->push(new CommandTurn(_swerveLib, 230));
		queue->push(new CommandVision(_swerveLib));
	}
}

void CommandManager::shootOnlyBin(queue<CommandBase*> *queue,robotTeam team, robotStation station) {

	if (station == TWO) {
		queue->push(new CommandDrive(_swerveLib, 10, 0));
		queue->push(new CommandTurn(_swerveLib, 90));
		queue->push(new CommandDrive(_swerveLib, 20, 0));
		queue->push(new CommandTurn (_swerveLib, 180));
		queue->push(new CommandDrive(_swerveLib, 20, 90));
		queue->push(new CommandTurn(_swerveLib, 270));
		queue->push(new CommandShoot(10, configShooterSpd(station), configShooterAng(station)));
		return;
	}
	if ((station == ONE && team == RED) || (station == THREE && team == BLUE)) {
		queue->push(new CommandDrive(_swerveLib, 20, 0));
		queue->push(new CommandTurn(_swerveLib, 270));
		queue->push(new CommandDrive(_swerveLib, 20, 0));
		queue->push(new CommandTurn (_swerveLib, 180));
		queue->push(new CommandDrive(_swerveLib, 20, 0));
		queue->push(new CommandTurn(_swerveLib, 90));
		queue->push(new CommandShoot(10, configShooterSpd(station), configShooterAng(station)));
		} else {
			queue->push(new CommandDrive(_swerveLib, 20, 0));
			queue->push(new CommandTurn(_swerveLib, 90));
			queue->push(new CommandDrive(_swerveLib, 20, 0));
			queue->push(new CommandTurn (_swerveLib, 180));
			queue->push(new CommandDrive(_swerveLib, 20, 0));
			queue->push(new CommandTurn(_swerveLib, 270));
			queue->push(new CommandShoot(10, configShooterSpd(station), configShooterAng(station)));
		}

}

double CommandManager::configShooterSpd(robotStation RS)
{
	double shooterSpd = 0;
	if(RS == ONE) shooterSpd = 3700;
	else if (RS == TWO) shooterSpd = 3900;
	else if (RS == THREE) shooterSpd = 4600;

	return shooterSpd;
 }

double CommandManager::configShooterAng(robotStation RS)
{
	double shooterAng = 0;
	if(RS == ONE) shooterAng = .4;
	else if (RS == TWO) shooterAng = .6;
	else if (RS == THREE) shooterAng = 1;

	return shooterAng;
}

CommandManager::~CommandManager() {
	// TODO Auto-generated destructor stub
}

