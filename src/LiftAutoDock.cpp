		printf("bA RE: ,%d\n", cntl1->bA->RE);
		if (cntl1->bA->RE == true){
			autoGearStateMachine = TARGET_AQUIRED;
		}

		switch (autoGearStateMachine) {
		case DO_NOTHING:
			//wait for the autolock button to be pressed
			x = 0;
			if (cntl1->bA->RE == true) {
				x = 0;
				printf("STARTING!!!\n");
				liftCenterDistance = /*wherever the distance between the centers of the tape pieces are*/0;
				if (liftCenterDistance/*vision system sees a "lift"*/ != 0) {
					autoGearStateMachine = TARGET_AQUIRED;
				}
			}
			break;
		case TARGET_AQUIRED:
			printf("STARTING!!!!!\n");
			//Get the current distance between the lifts
			liftCenterDistance = /*wherever the distance between the centers of the tape pieces are*/0;
			autoGearStateMachine = TURN_TO_SQUARE;
			if (cntl1->bA->State == true) autoGearStateMachine = DO_NOTHING;
			break;
		case HORIZONTAL_LINEUP:
			//Line up horizontally to the lift. Previous angle stays the same
			liftCenterDistance = /*wherever the distance between the centers of the tape pieces are*/0;
			if (x/*angle from center of picture horizontally to spring*/ > 5) {
				swerveLib->calcWheelVect(0, 0.5, 0);
			} else if (x/*angle from center of picture horizontally to spring*/ < 5) {
				swerveLib->calcWheelVect(0, -0.5, 0);
			}

			if (0/*angle from center of picture to spring*/ == 0) {
				autoGearStateMachine = TURN_TO_SQUARE;
			}
			if (cntl1->bA->State == true) autoGearStateMachine = DO_NOTHING;
			break;
		case TURN_TO_SQUARE:
			//Turn to be square against the lift that is being targeted
			liftCenterDistance = /*wherever the distance between the centers of the tape pieces are*/0;
			if (0/*distance in pixels from center of picture to spring*/ > 0) {
				swerveLib->calcWheelVect(0, 0, 0.5);
			} else if (0/*distance in pixels from center of picture horizontally to spring*/ < 0) {
				swerveLib->calcWheelVect(0, 0, -0.5);
			}
			if (0/*distance in pixels from center of picture horizontally to spring*/ == 0) {
				autoGearStateMachine = DRIVE_TO_SPRING;
			}
			if (cntl1->bA->State == true) autoGearStateMachine = DO_NOTHING;
			break;
		case DRIVE_TO_SPRING:
			//Drive forward to the point where the pilot can take the gear
			liftCenterDistance += /*wherever the distance between the centers of the tape pieces are*/0;
			if (x/*distance in pixels from center of picture horizontally to spring*/ != 0) autoGearStateMachine = HORIZONTAL_LINEUP;
			if (liftCenterDistance < 10) { //value here is subject to change with the robot
				swerveLib->calcWheelVect(0.5, 0, 0);
			} else if (liftCenterDistance == 10) autoGearStateMachine = DONE;
			if (cntl1->bA->State == true) autoGearStateMachine = DO_NOTHING;
			break;
		case DONE:
			//Keep the robot still for a bit so the pilot can safely take the gear out
			autoGearStateMachine = DO_NOTHING;
			break;
		}