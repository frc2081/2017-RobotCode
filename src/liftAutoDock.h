
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
		
		//Cals
		double horzLineUpDrvPwrHigh = .4;
		double horzLineUpDrvPwrLow = .2;
		double driveToLiftPwr = .4;
		double driveToLiftRot = -0.05;
		int horzLineUpTolerance = 10; //Target max distance from center of lift to center of image
		int distToLiftGoal = 300; //When moving toward the lift peg for final docking, how far apart
									//the targets have to be to indicate the robot has reached the goal
		int centerHoldAngle = 0;
									
		double centerHoldFactor = -0.2;

		int pegDistToImgCenter;
		int targetDistApart;
		
		void zeroDrive();
	
	public:
		liftAutoDock();
		void calcLiftAutoDock(bool autoDockCommand, bool targetLockStatus, int leftTargetDisanceToImageCenter, int rightTargetDistanceToImageCenter);
		double getLADDrvMagCmd();
		double getLADDrvAngCmd();
		double getLADDrvRotCmd();
};
