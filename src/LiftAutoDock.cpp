#include "liftAutoDock.h"
#include <iostream>

liftAutoDock::liftAutoDock()
{
	contourTable = NetworkTable::GetTable("GRIP/Contours");
	targetLock = false;
	leftTargetDistToImgCenter = 0;
	rightTargetDistToImgCenter = 0;
	liftAutoDockCmd = false;
	liftAutoDockState = DO_NOTHING;
	
	gearDeployCmd = false;
	gearDeployTime = 0;

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

bool liftAutoDock::getLADGearDeployCmd(){
	return gearDeployCmd;
}

void liftAutoDock::zeroDrive(){
	drvAngCmd = 0;
	drvMagCmd = 0;
	drvRotCmd = 0;
}

void liftAutoDock::prepVisionData(){
			//VISION CODE FOR LIFT AUTO DOCK
	std::vector<double> contourHeights = contourTable->GetNumberArray("height", llvm::ArrayRef<double>());
	std::vector<double> contourWidths = contourTable->GetNumberArray("width", llvm::ArrayRef<double>());
	std::vector<double>contourAreas = contourTable->GetNumberArray("area", llvm::ArrayRef<double>());
	std::vector<double> contourCenterXs = contourTable->GetNumberArray("centerX", llvm::ArrayRef<double>());
	std::vector<double> contourCenterYs = contourTable->GetNumberArray("centerY", llvm::ArrayRef<double>());

	//development code only, can remove later
	if(contourCenterXs.size() > 1 && contourCenterYs.size() > 1) {
		//SmartDashboard::PutNumber("First Contour Center X Pos: ", contourCenterXs[0]);
		//SmartDashboard::PutNumber("Second Contour Center X Pos: ", contourCenterXs[1]);
		//SmartDashboard::PutNumber("First Contour Center Y Pos: ", contourCenterYs[0]);
		//SmartDashboard::PutNumber("Second Contour Center Y Pos: ", contourCenterYs[1]);
		//SmartDashboard::PutNumber("Contour Y Center Delta: ", abs(contourCenterYs[1] - contourCenterYs[0]));

		if(abs(contourCenterYs[0] - contourCenterYs[1]) < liftCenterMaxYDiff){ //Any two contours left at this point with Y centers near each other are probably the lift targets
			liftTargetLeft = 0;
			liftTargetRight = 1;
			targetLock = true;
			leftTargetDistToImgCenter = contourCenterXs[0] - (liftImageWidth/2 -2);
			rightTargetDistToImgCenter = contourCenterXs[1] - (liftImageWidth/2 -2);
		}else {targetLock = false;}
	}else {targetLock = false;}
}

void liftAutoDock::calcLiftAutoDock(bool autoDockCommand){

	//Zero out all drive commands to prevent values from previous executions from being used
	zeroDrive();
	liftAutoDockCmd = autoDockCommand;
	prepVisionData();
	
	//Abort the entire state machine if the auto dock command is removed or if the target is not found/lost during docking
	if(liftAutoDockCmd == false) {
		liftAutoDockState = DO_NOTHING;	
		//printf("AutoDock: No Command ");
	} else if(liftAutoDockCmd == true && targetLock == false){
		printf("AutoDock: No Target Found. Gear delay %.2f", gearDeployTime);

		//liftAutoDockState = DO_NOTHING;

		gearDeployTime++;
		if (gearDeployTime > gearDeployDelay && gearDeployTime < gearDeployDuration) { gearDeployCmd = true; liftAutoDockState = DO_NOTHING;}
		else if (gearDeployTime > gearDeployDuration) { liftAutoDockState = DO_NOTHING; }
		else {gearDeployCmd = false;}
	}
	
	//Calculate all info about the lift target positions that is needed to guide the robot
	if(targetLock == true){
		pegDistToImgCenter = (leftTargetDistToImgCenter + rightTargetDistToImgCenter) / 2;
		targetDistApart = abs(-leftTargetDistToImgCenter + rightTargetDistToImgCenter);
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
				if(targetDistApart < driveToListSlowDownVerDist ) drvMagCmd = driveToLiftPwrHigh;
				else drvMagCmd = driveToLiftPwrLow;
			} /*else {
				drvMagCmd = 0;
				drvRotCmd = 0;
				liftAutoDockState = DONE;
			}*/
			break;	
			
		case DONE:
			printf("AutoDock: Done ");
			break;
		}

		//if(liftAutoDockCmd == true) {printf("HorzDist %i VertDist %i Mag %f Ang %f Rot %f Gear%d\n", pegDistToImgCenter, targetDistApart, drvMagCmd, drvAngCmd, drvRotCmd, gearDeployCmd);}
	}
	

		
