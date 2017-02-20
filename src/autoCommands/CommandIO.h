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
		double autoAimAng;
		double autoIntakePwr;

		commandOutput() {
			autoAng = 0;
			autoSpeed = 0;
			autoRot = 0;
			autoShooterSpd = 0;
			autoLoadSpd = 0;
			autoIntakePwr = 0;
			autoAimAng = .4;
		}

		commandOutput(double mag, double ang, double rot) {
			autoAng = ang;
			autoSpeed = mag;
			autoRot = rot;
			autoShooterSpd = 0;
			autoLoadSpd = 0;
			autoIntakePwr = 0;
			autoAimAng = .4;
		}

		commandOutput(double shootSpd, double rotSpd) {
			autoAng = 0;
			autoSpeed = 0;
			autoRot = 0;
			autoIntakePwr = 0;
			autoShooterSpd = shootSpd;
			autoLoadSpd = rotSpd;
			autoAimAng = .4;
		}

		commandOutput(double mag, double ang, double rot, double shootSpd, double rotSpd, double aimAng, double intakePwr){
			autoAng = ang;
			autoSpeed = mag;
			autoRot = rot;
			autoShooterSpd = shootSpd;
			autoLoadSpd = rotSpd;
			autoAimAng = aimAng;
			autoIntakePwr = intakePwr;
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
