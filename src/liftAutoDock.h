#include "WPIlib.h"


class liftAutoDock
{
	private:
		enum liftAutoDockStates {
		//Driver has not yet engaged the autolock
		DO_NOTHING,
		//Make sure the target is in the camera's field of vision
		TARGET_AQUIRED,
		//Make the robot square against the desired target
		TURN_TO_SQUARE,
		//Line the robot up to drive straight into the spring
		HORIZONTAL_LINEUP,
		//Drive to the proper distance to unload the spring
		DRIVE_TO_SPRING,
		//Wait a few seconds, then let normal drive controls work again
		DONE
		};
		
		liftAutoDockStates liftAutoDockState;
		
		//inputs and outputs
		bool targetLock;
		int leftTargetDistToImgCenter, rightTargetDistToImgCenter;
		bool liftAutoDockCmd;
		double drvRotCmd, drvAngCmd, drvMagCmd;
		bool gearDeployCmd;
		double gearDeployTime;

		
		//Cals
		double horzLineUpDrvPwrHigh = .25;
		double horzLineUpDrvPwrLow = .25;
		double driveToLiftPwrHigh = .4;
		double driveToLiftPwrLow = .3;
		double driveToListSlowDownVerDist = 100;
		double driveToLiftRot = -0.05;
		int horzLineUpTolerance = 20; //Target max distance from center of lift to center of image
		int distToLiftGoal = 300; //When moving toward the lift peg for final docking, how far apart
									//the targets have to be to indicate the robot has reached the goal
		double gearDeployDelay = 100;
		double gearDeployDuration = gearDeployDelay + 15; //duration of time to run gear deploy motor

		//IMAGE PROCESSING CALS
		//Cals are for a 320x240 image
		int liftImageWidth = 320;
		int liftImageHeight = 240;
		double liftMaxRatio = 7;
		double liftMinRatio = 1.5;
		int liftCenterMaxYDiff = 15;
		int liftCenterMaxYPos = 200;
		int liftCenterMaxXDist = 300;
		int liftCenterMinXDiff = 50;

		int centerHoldAngle = 0;
									
		double centerHoldFactor = -0.4;

		int pegDistToImgCenter;
		int targetDistApart;
		int liftTargetLeftContour;
		int liftTargetRightContour;
		int liftCenterDistance;
		int liftTargetLeft;
		int liftTargetRight;
		
		void zeroDrive();

		std::shared_ptr<NetworkTable> contourTable;

		void prepVisionData();
	
	public:
		liftAutoDock();
		void calcLiftAutoDock(bool liftAutoDockCmd);
		double getLADDrvMagCmd();
		double getLADDrvAngCmd();
		double getLADDrvRotCmd();
		bool getLADGearDeployCmd();
};
