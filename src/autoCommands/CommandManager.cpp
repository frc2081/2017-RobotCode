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

	commands = queue<CommandBase*>();
	buildCommands(&commands, team, station, action);

	_swerveLib = swerveLib;

	commands.push(_doNothing = new CommandPause(-1));
	currCommand = NULL;
}

commandOutput CommandManager::tick(commandInput input) {

	if (currCommand == NULL || currCommand->isDone())
		currCommand = getNextCommand(input);

	return currCommand == NULL
		? commandOutput()
		: currCommand->tick(input);
}

CommandBase *CommandManager::getNextCommand(commandInput input) {

	// In the event that all prior commands have been popped off the queue, do nothing.
	if (commands.size() < 1)
		return _doNothing;

	CommandBase *nextCommand =  commands.front();
	commands.pop();

	//Need to add NULL protection here in case no command was added to the queue prior to the pop
	if (currCommand != NULL) delete currCommand;

	if(nextCommand == NULL)
	{
		printf("Received a null command from the queue.\n");
		return _doNothing;
	}

	printf("Retrieved a %s Command from the queue.\n", nextCommand->getCommandName());
	nextCommand->init(input);
	printf("Command Init Successful\n");
	return nextCommand;
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
	queue->push(new CommandPause(-1));
}

//ALL VALUES IN COMMANDS ARE PLACEHOLDERS FOR ACTUAL VALUES
void CommandManager::crossMidline(queue<CommandBase*> *queue,robotTeam team, robotStation station) {
	if (station == TWO) {
		queue->push(new CommandDrive(_swerveLib, 40, 0));
		queue->push(new CommandDrive(_swerveLib, 84, 270));
		queue->push(new CommandDrive(_swerveLib, 100, 0));
	} else { queue->push(new CommandPause(1)); }
}

void CommandManager::gearOnly(queue<CommandBase*> *queue,robotTeam team, robotStation station) {
	if (station == TWO) {
		queue->push(new CommandVision(_swerveLib));
		return;
	}
	if ((station == ONE && team == RED) || (station == THREE && team == BLUE)) {
		//forward 75 inches
		//turn 45 degrees right
			queue->push(new CommandDrive(_swerveLib, 75, 180));
			queue->push(new CommandTurn (_swerveLib, 340));
			queue->push(new CommandVision(_swerveLib));
		} else {
			queue->push(new CommandDrive(_swerveLib, 75, 180));
			queue->push(new CommandTurn (_swerveLib, 235));
			queue->push(new CommandVision(_swerveLib));
		}
}

void CommandManager::shootOnly(queue<CommandBase*> *queue,robotTeam team, robotStation station) {
	queue->push(new CommandShoot(15, configShooterSpd(station), configShooterAng(station)));
}

void CommandManager::gearAndShoot(queue<CommandBase*> *queue,robotTeam team, robotStation station) {

		queue->push(new CommandShoot(8, configShooterSpd(station), configShooterAng(station)));
		gearOnly(queue, team, station);
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
	if(RS == ONE) shooterSpd = 3000;
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

