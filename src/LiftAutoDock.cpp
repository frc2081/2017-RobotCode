#include "liftAutoDock.h"
#include <iostream>

double liftAutoDock::getLADDrvMagCmd(){
	return drvMagCmd;
}

double liftAutoDock::getLADDrvAngCmd(){
	return drvAngCmd;
}

double liftAutoDock::getLADDrvRotCmd(){
	return drvRotCmd;
}

liftAutoDock::liftAutoDock()
{
	targetLock = false;
	leftTargetDistToImgCenter = 0;
	rightTargetDistToImgCenter = 0;
	liftAutoDockCmd = false;
	drvRotCmd = 0;
	drvAngCmd = 0;
	drvMagCmd = 0;

	pegDistToImgCenter = 0;
	targetDistApart = 0;
}

void liftAutoDock::calcLiftAutoDock(bool autoDockCommand, bool targetLockStatus, int leftTargetDisanceToImageCenter, int rightTargetDistanceToImageCenter){

	liftAutoDockCmd = autoDockCommand;
	targetLock = targetLockStatus;
	leftTargetDistToImgCenter = leftTargetDisanceToImageCenter;
	rightTargetDistToImgCenter = rightTargetDistanceToImageCenter;
	
//Abort the entire state machine if the auto dock command is removed
//or if the target is not found/lost during docking
if(liftAutoDockCmd == false) {
	liftAutoDockState = DO_NOTHING;	
} else if(liftAutoDockCmd == true && targetLock == false){
	printf("AutoDock: No Target Found\n");
	liftAutoDockState = DO_NOTHING;	
}

//************Automatic lift docking state machine**************
switch (liftAutoDockState) {
	case DO_NOTHING:
		if (liftAutoDockCmd == true && targetLock == true) {
			liftAutoDockState = HORIZONTAL_LINEUP;
		}
		break;
		
	case HORIZONTAL_LINEUP:		
		pegDistToImgCenter = (leftTargetDistToImgCenter + rightTargetDistToImgCenter) / 2;
		printf("AutoDock: Horizontal Lineup. CurrDist %i\n", pegDistToImgCenter);
		
		if(pegDistToImgCenter > horzLineUpTolerance){
			drvAngCmd = 0; //Moving towards 0 degrees moves peg to the left in the image
			drvMagCmd = horzLineUpDrvPwr;
		} else if(pegDistToImgCenter < -horzLineUpTolerance){
			drvAngCmd = 180; //Moving towards 180 degrees moves peg to the right in the image
			drvMagCmd = horzLineUpDrvPwr;
		} else {
			drvMagCmd = 0;
			liftAutoDockState = DRIVE_TO_SPRING;
		}
		
		break;	
		
	case TURN_TO_SQUARE://not currently used
		break;	
		
	case DRIVE_TO_SPRING:
		targetDistApart = (-leftTargetDistToImgCenter + rightTargetDistToImgCenter);
		printf("AutoDock: Driving to Lift. CurrDist %i\n", targetDistApart);
		
		if(targetDistApart < distToLiftGoal){
			drvAngCmd = 270;
			drvMagCmd = driveToLiftPwr; 
		} else {
			drvMagCmd = 0;
			liftAutoDockState = DONE;
		}
		break;	
		
	case DONE:
		printf("AutoDock: Done\n");
		break;
	}	
}
		
