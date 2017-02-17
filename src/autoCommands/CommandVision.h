/*
 * CommandVision.h
 *
 *  Created on: Jan 31, 2017
 *      Author: FIRSTUser
 */

#ifndef SRC_AUTOCOMMANDS_COMMANDVISION_H_
#define SRC_AUTOCOMMANDS_COMMANDVISION_H_
#include "CommandBase.h"

class CommandVision : public CommandBase {
public:
	CommandVision(swervelib *swerveLib);
	virtual ~CommandVision();


	void init(commandInput input);

	commandOutput tick(commandInput input);
private:

};

#endif /* SRC_AUTOCOMMANDS_COMMANDVISION_H_ */
