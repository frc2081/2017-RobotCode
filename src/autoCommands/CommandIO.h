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
		double autoShooterSpd;
		double autoLoadSpd;

		commandOutput() {
			autoAng = 0;
			autoSpeed = 0;
			autoRot = 0;
			autoShooterSpd = 0;
		}

		commandOutput(double mag, double ang, double rot) {
			autoAng = ang;
			autoSpeed = mag;
			autoRot = rot;
		}

		commandOutput(double shootSpd, double rotSpd) {
			autoShooterSpd = shootSpd;
			autoLoadSpd = rotSpd;
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
		double shooterEnc;
	};

	enum robotTeam {
		NONE,
		BLUE,
		RED
	};

	enum robotStation {
		UNKNOWN,
		ONE,
		TWO,
		THREE
	};

	enum robotAction {
		NO_AUTO,
		CROSS_MIDLINE,
		GEAR_ONLY,
		SHOOT_ONLY,
		GEAR_AND_SHOOT,
		SHOOT_ONLY_BIN
	};


#endif /* SRC_AUTOCOMMANDS_COMMANDIO_H_ */
