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
//#include "Calibrations.h"

using namespace std;
class CommandManager {
public:
	CommandManager(swervelib *swerveLib, robotTeam, robotStation, robotAction);
	virtual ~CommandManager();
	commandOutput tick(commandInput input);

protected:
	CommandBase *getNextCommand(commandInput input);

private:
	swervelib *_swerveLib;

	queue<CommandBase*> commands;

	void buildCommands(queue<CommandBase*> *queue,robotTeam, robotStation, robotAction);

	CommandBase *currCommand;

	void crossMidline(queue<CommandBase*> *queue,robotTeam, robotStation);

	void gearOnly(queue<CommandBase*> *queue,robotTeam, robotStation);

	void shootOnly(queue<CommandBase*> *queue,robotTeam, robotStation);

	void gearAndShoot(queue<CommandBase*> *queue,robotTeam, robotStation);

	void shootOnlyBin(queue<CommandBase*> *queue,robotTeam, robotStation);

	double configShooterAng(robotTeam, robotStation);

	double configShooterSpd(robotTeam, robotStation);

	CommandBase *_doNothing;
};

#endif /* SRC_AUTOCOMMANDS_COMMANDMANAGER_H_ */
