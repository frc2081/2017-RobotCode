/*
 * CommandIO.h
 *
 *  Created on: Jan 30, 2017
 *      Author: FIRSTUser
 */

#ifndef SRC_AUTOCOMMANDS_COMMANDIO_H_
#define SRC_AUTOCOMMANDS_COMMANDIO_H_

	struct commandOutput {
		double autoAng;
		double autoSpeed;
		double autoRot;

		commandOutput() {
			autoAng = 0;
			autoSpeed = 0;
			autoRot = 0;
		}

		commandOutput(double mag, double ang, double rot) {
			autoAng = ang;
			autoSpeed = mag;
			autoRot = rot;
		}
	};

	struct  commandInput {
		double currentGyroReading;
		double LFWhlTurnEnc;
		double RFWhlTurnEnc;
		double RBWhlTurnEnc;
		double LBWhlTurnEnc;
		double LFWhlDrvEnc;
		double RFWhlDrvEnc;
		double RBWhlDrvEnc;
		double LBWhlDrvEnc;
	};



#endif /* SRC_AUTOCOMMANDS_COMMANDIO_H_ */
