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
	CommandVision();
	virtual ~CommandVision();


	void init();

	commandOutput tick(commandInput input);
private:

};

#endif /* SRC_AUTOCOMMANDS_COMMANDVISION_H_ */
