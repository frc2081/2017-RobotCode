/*
 * CommandShoot.h
 *
 *  Created on: Feb 15, 2017
 *      Author: FIRSTUser
 */

#ifndef SRC_AUTOCOMMANDS_COMMANDSHOOT_H_
#define SRC_AUTOCOMMANDS_COMMANDSHOOT_H_
#include "CommandBase.h"
#include <time.h>

class CommandShoot : public CommandBase {
public:
	CommandShoot(double fireTimeSec, double shooterSpd, double shooterAng);
	virtual ~CommandShoot();

	void init(commandInput input);
	commandOutput tick(commandInput input);
	const char* getCommandName();
private:
	time_t _initTime;

	double _toWaitSec;
	double _shootSpd;
	double _shootAng;

};

#endif /* SRC_AUTOCOMMANDS_COMMANDSHOOT_H_ */
