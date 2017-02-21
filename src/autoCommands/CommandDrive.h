/*
 * CommandDrive.h
 *
 *  Created on: Jan 31, 2017
 *      Author: FIRSTUser
 */

#ifndef SRC_AUTOCOMMANDS_COMMANDDRIVE_H_
#define SRC_AUTOCOMMANDS_COMMANDDRIVE_H_
#include "CommandBase.h"

class CommandDrive : public CommandBase {
public:
	//Distance to travel is in imperial units
	CommandDrive(swervelib *swerveLib, double toTravel, double directon);
	virtual ~CommandDrive();

	commandOutput tick(commandInput input);
	void init(commandInput input);
	const char* getCommandName();

protected:
	double checkDistance(commandInput input);
private:
	swervelib *_swerveLib;

	double LFWhlTurnEncInit;
	double RFWhlTurnEncInit;
	double LBWhlTurnEncInit;
	double RBWhlTurnEncInit;

	double LFWhlDrvEncInit;
	double RFWhlDrvEncInit;
	double LBWhlDrvEncInit;
	double RBWhlDrvEncInit;

	double _toTravel;
	double _direction;

	//infinite ducks
};

#endif /* SRC_AUTOCOMMANDS_COMMANDDRIVE_H_ */
