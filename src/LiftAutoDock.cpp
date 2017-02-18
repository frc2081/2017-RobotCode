#include "liftAutoDock.h"
#include <iostream>

liftAutoDock::liftAutoDock()
{
	targetLock = false;
	leftTargetDistToImgCenter = 0;
	rightTargetDistToImgCenter = 0;
	liftAutoDockCmd = false;
	
	pegDistToImgCenter = 0;
	targetDistApart = 0;
	
	zeroDrive();
}

double liftAutoDock::getLADDrvMagCmd(){
	return drvMagCmd;
}

double liftAutoDock::getLADDrvAngCmd(){
	return drvAngCmd;
}

double liftAutoDock::getLADDrvRotCmd(){
	return drvRotCmd;
}

void liftAutoDock::zeroDrive(){
	drvAngCmd = 0;
	drvMagCmd = 0;
	drvRotCmd = 0;
}

void liftAutoDock::calcLiftAutoDock(bool autoDockCommand, bool targetLockStatus, int leftTargetDisanceToImageCenter, int rightTargetDistanceToImageCenter){

	liftAutoDockCmd = autoDockCommand;
	targetLock = targetLockStatus;
	leftTargetDistToImgCenter = leftTargetDisanceToImageCenter;
	rightTargetDistToImgCenter = rightTargetDistanceToImageCenter;
	
	//Zero out all drive commands to prevent values from previous executions from being used
	zeroDrive();
	
	//Abort the entire state machine if the auto dock command is removed or if the target is not found/lost during docking
	if(liftAutoDockCmd == false) {
		liftAutoDockState = DO_NOTHING;	
	} else if(liftAutoDockCmd == true && targetLock == false){
		printf("AutoDock: No Target Found ");
		liftAutoDockState = DO_NOTHING;	
	}
	
	//Calculate all info about the lift target positions that is needed to guide the robot
	if(targetLock == true){
		pegDistToImgCenter = (leftTargetDistToImgCenter + rightTargetDistToImgCenter) / 2;
		targetDistApart = (-leftTargetDistToImgCenter + rightTargetDistToImgCenter);
	}

	//************Automatic lift docking state machine**************
	switch (liftAutoDockState) {
		case DO_NOTHING:
			if (liftAutoDockCmd == true && targetLock == true) {
				liftAutoDockState = HORIZONTAL_LINEUP;
			}
			break;
			
		case HORIZONTAL_LINEUP:	
			printf("AutoDock: Horz Lineup. ");
			
			if(abs(pegDistToImgCenter) > horzLineUpTolerance * 4) drvMagCmd = horzLineUpDrvPwrHigh;
			else drvMagCmd = horzLineUpDrvPwrLow;

			if(pegDistToImgCenter > horzLineUpTolerance){
				drvAngCmd = 0; //Moving towards 0 degrees moves peg to the left in the image
			} else if(pegDistToImgCenter < -horzLineUpTolerance){
				drvAngCmd = 180; //Moving towards 180 degrees moves peg to the right in the image
			} else {
				drvMagCmd = 0;
				liftAutoDockState = DRIVE_TO_SPRING;
			}
			break;	
			
		case TURN_TO_SQUARE://not currently used
			break;	
			
		case DRIVE_TO_SPRING:
			printf("AutoDock: Drv to Lift. ");	
			centerHoldAngle = pegDistToImgCenter * centerHoldFactor;
			
			if(targetDistApart < distToLiftGoal){
				drvAngCmd = 90 + centerHoldAngle;
				drvMagCmd = driveToLiftPwr; 
			} else {
				drvMagCmd = 0;
				drvRotCmd = 0;
				liftAutoDockState = DONE;
			}
			break;	
			
		case DONE:
			printf("AutoDock: Done ");
			break;
		}	
		printf("HorzDist %i VertDist %i Mag %f Ang %f Rot %f\n", pegDistToImgCenter, targetDistApart, drvMagCmd, drvAngCmd, drvRotCmd);
	}
	

		
