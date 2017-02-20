#include "Robot.h"

VictorSP *LFMotDrv;
VictorSP *LBMotDrv;
VictorSP *RFMotDrv;
VictorSP *RBMotDrv;
VictorSP *LFMotTurn;
VictorSP *LBMotTurn;
VictorSP *RFMotTurn;
VictorSP *RBMotTurn;
VictorSP *ClimbMotDrv1;
VictorSP *ClimbMotDrv2;
VictorSP *ClimbMotDrv3;
VictorSP *ballLoad;
VictorSP *ballFeederMot;
VictorSP *ballShooterMot;

Servo *shooterAimServo;

AnalogPotentiometer *LFEncTurn;
AnalogPotentiometer *LBEncTurn;
AnalogPotentiometer *RFEncTurn;
AnalogPotentiometer *RBEncTurn;

Encoder *LFEncDrv;
Encoder *RFEncDrv;
Encoder *LBEncDrv;
Encoder *RBEncDrv;
Encoder *shooterEnc;

cntl *cntl1;
cntl *cntl2;

swervelib *swerveLib;

PIDController *LFPID;
PIDController *LBPID;
PIDController *RFPID;
PIDController *RBPID;
PIDController *shooterPID;

float currentAngle;
float currentFacing;
float comAng, comMag, comRot;
double currAng1, currAng2, currAng3, currAng4;

//Declare all control variables
bool runShooter;
double shooterPower;
double feederSpeed;
double shooterAimLocation;
double climbSpeed;

commandInput autoInput;
commandOutput autoOutput;

double Encoder::PIDGet(){ return this->GetRate();}

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() {

		//Create a new camera server in it's own thread
		//ALWAYS put things like cameras in their own thread so if they crash and burn it doesn't take down the wholw control system
        std::thread visionThread(VisionThread);
        visionThread.detach();
		
		AD = new liftAutoDock();
		autoEnable = new AnalogInput(4);
		autoAction = new AutoSelector(7);
		autoFieldPosition = new AutoSelector(6);

		//Instantiate the joysticks according to the controller class
		cntl1 = new cntl(0, .2);
		cntl2 = new cntl(1, .2);

		//Instantiate the swerve calculations library
		swerveLib = new swervelib(27, 23.5);

		gyroManagerRun = gyroManager::Get();
		gyroManagerRun->start();

		//Instantiate the encoders
		LFEncTurn = new AnalogPotentiometer(0, 360, 0);
		RFEncTurn = new AnalogPotentiometer(2, 360, 0);
		LBEncTurn = new AnalogPotentiometer(1, 360, 0);
		RBEncTurn = new AnalogPotentiometer(3, 360, 0);

		LFEncDrv = new Encoder(4, 5, false);
		RFEncDrv = new Encoder(2, 3, false);
		LBEncDrv = new Encoder (6, 7, false);
		RBEncDrv = new Encoder (8, 9, false);
		shooterEnc = new Encoder(0, 1, false);

		//Instantiate the motors based on where they are plugged in
		LFMotTurn = new VictorSP(8);
		RFMotTurn = new VictorSP(12);
		LBMotTurn = new VictorSP(3);
		RBMotTurn = new VictorSP(6);

		LFMotDrv = new VictorSP(9);
		RFMotDrv = new VictorSP(13);
		LBMotDrv = new VictorSP(4);
		RBMotDrv = new VictorSP(7);

		ClimbMotDrv1 = new VictorSP(0);
		ClimbMotDrv2 = new VictorSP(1);
		ClimbMotDrv3 = new VictorSP(2);

		//Located on the MXP expansion board
		ballLoad = new VictorSP(15);
		ballFeederMot = new VictorSP(11);
		ballShooterMot = new VictorSP(5);
		shooterAimServo = new Servo(14);

		LFEncDrv->SetDistancePerPulse(drvWhlDistPerEncCnt);
		RFEncDrv->SetDistancePerPulse(drvWhlDistPerEncCnt);
		LBEncDrv->SetDistancePerPulse(drvWhlDistPerEncCnt);
		RBEncDrv->SetDistancePerPulse(drvWhlDistPerEncCnt);
		shooterEnc->SetDistancePerPulse(shooterEncCntPerRev);

		//Instantiate the PID controllers to their proper values
		LFPID = new PIDController(turnMotorP, turnMotorI, turnMotorD, LFEncTurn, LFMotTurn, period);
		RFPID = new PIDController(turnMotorP, turnMotorI, turnMotorD, RFEncTurn, RFMotTurn, period);
		LBPID = new PIDController(turnMotorP, turnMotorI, turnMotorD, LBEncTurn, LBMotTurn, period);
		RBPID = new PIDController(turnMotorP, turnMotorI, turnMotorD, RBEncTurn, RBMotTurn, period);

		//TODO: Put all this into a function
		LFPID->SetInputRange(0,360);
		LFPID->SetOutputRange(-1,1);
		LFPID->SetContinuous();
		LFPID->Enable();

		RFPID->SetInputRange(0,360);
		RFPID->SetOutputRange(-1,1);
		RFPID->SetContinuous();
		RFPID->Enable();

		LBPID->SetInputRange(0,360);
		LBPID->SetOutputRange(-1,1);
		LBPID->SetContinuous();
		LBPID->Enable();

		RBPID->SetInputRange(0,360);
		RBPID->SetOutputRange(-1,1);
		RBPID->SetContinuous();
		RBPID->Enable();

		LFEncDrv->Reset();
		RFEncDrv->Reset();
		LBEncDrv->Reset();
		RBEncDrv->Reset();

		//NOTE: Shooter speed is revolutions per SECOND, not RPM
		//VERY important that min command from PID be set to 0....otherwise the instant that the motor
		//exceeded the target speed, it would attempt to reverse the shooter wheel!
		shooterPID = new PIDController(shooterSpdP, shooterSpdI, shooterSpdD, shooterEnc, ballShooterMot, period);
		shooterPID->SetInputRange(0,shooterMaxRevPerSec);
		shooterPID->SetOutputRange(0,1);
		shooterPID->SetSetpoint(0);
		shooterPID->Enable();

		//Init all control variables to safe states
		runShooter = false;
		feederSpeed = 0;
		climbSpeed = 0;
		shooterAimLocation = shooterAngNearShot;
		autoDockCmd = false;
		shooterToggle = 0;
		shooterSelection = 0;
		shooterPower = 0;
		
		//Remove this later when shooter power levels have been determined
		SmartDashboard::PutNumber("Shooter Speed Adjust: ", 0);
		SmartDashboard::PutNumber("Shooter Setpoint: ",0);

		SmartDashboard::PutNumber("LF Offset: ", LFOffset);
		SmartDashboard::PutNumber("RF Offset: ", RFOffset);
		SmartDashboard::PutNumber("LB Offset: ", LBOffset);
		SmartDashboard::PutNumber("RB Offset: ", RBOffset);
	}

	void AutonomousInit() override {
		if(autoEnable->GetVoltage() <= 1) {
			printf("******************AUTO DISABLED!*********************");
			return; //if auto enable switch is off, do nothing
		}
		
		/*	AutoAction values
			0 = do nothing
			1 = cross the midline only - probably will not implement
			2 = place gear only
			3 = shoot only
			4 = shoot, place gear
			5 - 8 = do nothing
		*/
		robotAction RA = autoAction->getAction();
		robotStation RS = autoFieldPosition->getFieldPosition();
		
		DriverStation::Alliance matchAlliance;
		robotTeam matchTeam;
		matchAlliance = DriverStation::GetInstance().GetAlliance();
		if(matchAlliance == DriverStation::Alliance::kBlue) matchTeam = BLUE;
		else matchTeam = RED;
		
		robotStation matchStation;
		int driverStationNumber = DriverStation::GetInstance().GetLocation();
		if(driverStationNumber == 1) matchStation = ONE;
		else if(driverStationNumber == 2) matchStation = TWO;
		else matchStation = THREE;

		printf("\n\n\n***********************INIT AUTONOMOUS MODE*************************\n");
		printf("Auto Action: %i\n", RA);
		printf("Field Position: %i\n", RS );
		printf("Alliance: %i\n", matchAlliance);
		printf("Driver Station Number: %i\n", matchStation);
		printf("*********************************************************************\n\n\n");

		autoCom = new CommandManager(swerveLib, matchTeam, RS, RA);
	}

	void AutonomousPeriodic() {
		if(autoEnable->GetVoltage() <= 1) return; //if auto enable switch is off, do nothing
		
		autoInput.LFWhlDrvEnc = LFEncDrv->GetDistance();
		autoInput.RFWhlDrvEnc = RFEncDrv->GetDistance();
		autoInput.LBWhlDrvEnc = LBEncDrv->GetDistance();
		autoInput.RBWhlDrvEnc = RBEncDrv->GetDistance();

		autoInput.LFWhlTurnEnc = LFEncTurn->Get();
		autoInput.RFWhlTurnEnc = RFEncTurn->Get();
		autoInput.LBWhlTurnEnc = LBEncTurn->Get();
		autoInput.RBWhlTurnEnc = RBEncTurn->Get();

		autoInput.currentGyroReading = gyroManagerRun->getLastValue();

		autoOutput = autoCom->tick(autoInput);

		swerveLib->calcWheelVect(autoOutput.autoSpeed, autoOutput.autoAng, autoOutput.autoRot);
		shooterPID->SetSetpoint(autoOutput.autoShooterSpd / 60);
		ballFeederMot->Set(autoOutput.autoLoadSpd);
		shooterAimServo->Set(autoOutput.autoAimAng);
		ballLoad->Set(autoOutput.autoIntakePwr);

		LFPID->SetSetpoint(swerveLib->whl->angleLF);
		RFPID->SetSetpoint(swerveLib->whl->angleRF);
		LBPID->SetSetpoint(swerveLib->whl->angleLB);
		RBPID->SetSetpoint(swerveLib->whl->angleRB);

		LFMotDrv->Set(swerveLib->whl->speedLF);
		RFMotDrv->Set(swerveLib->whl->speedRF);
		LBMotDrv->Set(swerveLib->whl->speedLB);
		RBMotDrv->Set(swerveLib->whl->speedRB);

	}

	void TeleopInit() {
		shooterPID->SetSetpoint(0);
	}

	void TeleopPeriodic() {
		//Update the joystick values
		cntl1->UpdateCntl();
		cntl2->UpdateCntl();

		//Soft limit the rotational speed so the gyro is not overloaded
		cntl1->RX *= .9;

		//Gyro needs to be mounted in center of robot otherwise it will not work properly	
		//Limit the gyro to a 360 degree output
		currentFacing = fabs(gyroManagerRun->getLastValue());
		if (currentFacing >= 360) currentFacing = ((int)currentFacing % 360);

		//Store current angles of the swerve wheels so we can calculate the angle delta later
		//This is so we can implement efficient swerve (no wheel ever turns more than 90 degrees)
		currAng1 = swerveLib->whl->angleRF;
		currAng2 = swerveLib->whl->angleLF;
		currAng3 = swerveLib->whl->angleLB;
		currAng4 = swerveLib->whl->angleRB;

		//Determine Lift auto docking command
		if (cntl1->bA->State == true) autoDockCmd = true;
		else autoDockCmd = false;
		
		AD->calcLiftAutoDock(autoDockCmd);
	
		//If driver is commanding auto-align, it controls the drive train, otherwise, use joystick inputs
		if(autoDockCmd == true){
			comAng = AD->getLADDrvAngCmd();
			comMag = AD->getLADDrvMagCmd();
			comRot = AD->getLADDrvRotCmd();
			swerveLib->calcWheelVect(comMag, comAng, comRot);
		} else {
		//Calculate commanded robot motion from the drive controller stick
		//Converts the two axes of the stick into a vector of angle comAng and magnitude comMag
		comAng = (atan2(-cntl1->LX, cntl1->LY) * 180/PI);// + currentFacing;
		comMag = sqrt(pow(cntl1->LX, 2) + pow(cntl1->LY, 2));
		comRot = cntl1->RX;

			//Calculate the proper values for the swerve drive motion. If there are no inputs, keep the wheels in their previous position
			if (cntl1->LX != 0 || cntl1->LY != 0 || cntl1->RX != 0) {
				swerveLib->calcWheelVect(comMag, comAng, comRot);
			} else {
				swerveLib->whl->speedRF = 0;
				swerveLib->whl->speedLF = 0;
				swerveLib->whl->speedLB = 0;
				swerveLib->whl->speedRB = 0;
			}
		}

		//Intelligent Swerve imeplementation - prevents any wheel from having to rotate more than 90 degrees to carry out a command
		if (fabs(swerveLib->whl->angleRF - currAng1) > 90 && fabs(swerveLib->whl->angleRF - currAng1) < 270) {
			swerveLib->whl->angleRF = ((int)swerveLib->whl->angleRF + 180) % 360;
			swerveLib->whl->speedRF *= -1;
		}

		if (fabs(swerveLib->whl->angleLF - currAng2) > 90 && fabs(swerveLib->whl->angleLF - currAng2) < 270) {
			swerveLib->whl->angleLF = ((int)swerveLib->whl->angleLF + 180) % 360;
			swerveLib->whl->speedLF *= -1;
		}

		if (fabs(swerveLib->whl->angleLB - currAng3) > 90 && fabs(swerveLib->whl->angleLB - currAng3) < 270) {
			swerveLib->whl->angleLB = ((int)swerveLib->whl->angleLB + 180) % 360;
			swerveLib->whl->speedLB *= -1;
		}

		if (fabs(swerveLib->whl->angleRB - currAng4) > 90 && fabs(swerveLib->whl->angleRB - currAng4) < 270) {
			swerveLib->whl->angleRB = ((int)swerveLib->whl->angleRB + 180) % 360;
			swerveLib->whl->speedRB *= -1;
		}

		//Set the PID controllers to angle the wheels properly
		RFPID->SetSetpoint(swerveLib->whl->angleRF);
		LFPID->SetSetpoint(swerveLib->whl->angleLF);
		RBPID->SetSetpoint(swerveLib->whl->angleRB);
		LBPID->SetSetpoint(swerveLib->whl->angleLB);

		//Set the wheel speed based on what the calculations from the swervelib
		LFMotDrv->Set(swerveLib->whl->speedLF);
		RFMotDrv->Set(swerveLib->whl->speedRF);
		RBMotDrv->Set(swerveLib->whl->speedRB);
		LBMotDrv->Set(swerveLib->whl->speedLB);

		//*********WINCH***********
		//Climbing is locked out unless the Y button of the drive controller is also held
		//This is to prevent accidental command of the winch before the robot is ready to climb
		climbSpeed = cntl1->RTrig;

		if(cntl1->bY->State == true){
			ClimbMotDrv1->Set(-climbSpeed); //Climb commands are negative to run the winch in the mechanically correct direction
			ClimbMotDrv2->Set(-climbSpeed);
			ClimbMotDrv3->Set(-climbSpeed);

			RFPID->SetSetpoint(winchDriveAngle);
			LFPID->SetSetpoint(winchDriveAngle);
			RBPID->SetSetpoint(winchDriveAngle);
			LBPID->SetSetpoint(winchDriveAngle);

			LFMotDrv->Set(climbSpeed*winchDriveFactor);
			RFMotDrv->Set(climbSpeed*winchDriveFactor);
			RBMotDrv->Set(climbSpeed*winchDriveFactor);
			LBMotDrv->Set(climbSpeed*winchDriveFactor);

		} else {
			ClimbMotDrv1->Set(0);
			ClimbMotDrv2->Set(0);
			ClimbMotDrv3->Set(0);
		}

		//*********INTAKE*********
		//Get ball intake command and set output
		if (cntl2->bLB->State == true) ballLoad->Set(fuelIntakeSpeedReverse);
		else if (cntl2->bRB->State == true) ballLoad->Set(fuelIntakeSpeedForward);
		else ballLoad->Set(0);
		//SmartDashboard::PutBoolean("BallLoadSpeed: ", cntl2->bY->State);

		//*********FEEDER********
		//Get the ball feeder command and set output. The left trigger is subtracted because it runs the feeder in reverse
		feederSpeed = cntl2->RTrig - cntl2->LTrig;
		ballFeederMot->Set(feederSpeed);

		//*********SHOOTER********
		//Toggle the shooter on and off with the start button
		if (cntl2->bStart->RE == true) { shooterToggle = true;}
		else {shooterToggle = false; }

		if (cntl2->bBack->RE) {runShooter = !runShooter;}

		SmartDashboard::PutNumber("Shooter Setpoint: ",shooterPID->GetSetpoint() / 60);
		double shooterSpeedAdjust = SmartDashboard::GetNumber("Shooter Speed Adjust: ", 0);

		//SHOOTER CONSTANT POWER CODE
		if (shooterToggle == true) {
			shooterSelection++;
			if (shooterSelection >= 3) {shooterSelection = 0;}
			if (shooterSelection == 0) {shooterPower = 0;}
			if (shooterSelection == 1) {shooterPID->SetSetpoint(shooterSpdNearShot); shooterAngle = shooterAngNearShot;}
			if (shooterSelection == 2) {shooterPID->SetSetpoint(shooterSpdFarShot); shooterAngle = shooterAngFarShot;}
		}
		//if(runShooter == false){ballShooterMot->Set(shooterPowerAdjust); }

		//SHOOTER PID CODE
		shooterPID->SetPID(shooterSpdP,shooterSpdI,shooterSpdD,0);
		if (runShooter == true) { shooterPID->SetSetpoint(shooterSpeedAdjust /60); }
		else shooterPID->SetSetpoint(0);

		//Aim the shooter

		//remove this for prod release
		//Each button press moves the shooter up or down by a fixed increment within the limits of the servo command
		//limiting the command here is needed because otherwise there is nothing stopping the command from
		//being incremented outside of the valid command range of the servo
		if (cntl2->bA->RE == true) shooterAimLocation += shooterAimIncrement;
		if (cntl2->bB->RE == true) shooterAimLocation -= shooterAimIncrement;
		if (shooterAimLocation > 1) shooterAimLocation = 1;
		if (shooterAimLocation < 0) shooterAimLocation = 0;
		shooterAimServo->Set(shooterAimLocation);

		//Debug print statements
		SmartDashboard::PutNumber("LF: ", LFEncDrv->Get());
		SmartDashboard::PutNumber("RF: ", RFEncDrv->Get());
		SmartDashboard::PutNumber("LB: ", LBEncDrv->Get());
		SmartDashboard::PutNumber("RB: ", RBEncDrv->Get());
		
		SmartDashboard::PutNumber("Shooter Aim Position: ", shooterAimLocation);
		SmartDashboard::PutNumber("Shooter Speed RPM: ", shooterEnc->GetRate()*60);
		
		SmartDashboard::PutNumber("LF Distance: ", LFEncDrv->Get());
		SmartDashboard::PutNumber("RF Distance: ", RFEncDrv->Get());
		SmartDashboard::PutNumber("LB Distance: ", LBEncDrv->Get());
		SmartDashboard::PutNumber("RB Distance: ", RBEncDrv->Get());
		
		//SmartDashboard::PutNumber("Left Target Center Pos: ", contourCenterYs[liftTargetLeft]);
		//SmartDashboard::PutNumber("Right Target Center Pos: ", contourCenterYs[liftTargetRight]);
		//SmartDashboard::PutNumber("Left Target Dist to Center: ", liftTargetLeftDistToImgCenter);
		//SmartDashboard::PutNumber("Right Target Dist to Center: ",liftTargetRightDistToImgCenter);
		//SmartDashboard::PutBoolean("Lift Target Acquired: ", liftTargetAcquired);
		
		//printf("LFEnc: %f RFEnc: %f LBEnc: %f RBEnc: %f Gyro %f\n", LFEncDrv->GetDistance(), RFEncDrv->GetDistance(), LBEncDrv->GetDistance(),RBEncDrv->GetDistance(), gyroManagerRun->getLastValue() );

		/*printf("%.2f, %.2f, %.2f, %.2f\n", swerveLib->whl->angleRF,
				swerveLib->whl->angleLF, swerveLib->whl->angleLB, swerveLib->whl->angleRB);
		printf("%.2f, %.2f, %.2f, %.2f\n\n", swerveLib->whl->speedRF, swerveLib->whl->speedLF,
				swerveLib->whl->speedLB, swerveLib->whl->speedRB);
		printf("%.2f, %.2f, %.2f\n\n", cntl1->LX, cntl1->LY, cntl1->RX);
		printf("%.5f, %.5f\n\n", gyroManagerRun->getLastValue(), currentFacing);
		std::cout << cntl1->bA->RE << ClimbMotDrv1->Get() << "\n\n";
		printf("%.2f, %.2f\n", feederSpeed, ballFeederMot->Get());
		printf("%d, %.2f\n", runShooter, ballShooterMot->Get());
		printf("%.2f\n", ballLoad->Get());
		printf("%.2i\n\n", shooterEnc->Get());*/
	}

	void TestPeriodic() {
	}

	void DisabledPeriodic() {

	//	robotAction RA = autoAction->getAction();
		//robotStation RS = autoFieldPosition->getFieldPosition();

		//printf("Auto Action: %i\n", RA);
	//	printf("Field Position: %i\n", RS);

		//printf("AutoAction: %f\n\n\n", autoEnable->GetVoltage());
		//printf("LFEnc Turn: %.2f\n", LFEncTurn->Get());
		//printf("RFEnc Turn: %.2f\n", RFEncTurn->Get());
		//printf("LBEnc Turn: %.2f\n", LBEncTurn->Get());
		//printf("RBEnc Turn: %.2f\n", RBEncTurn->Get());
		//printf("Gyro: %.5f\n\n", gyroManagerRun->getLastValue());

		//printf("test result: %f\n", SmartDashboard::GetNumber("A", 0));

	}
	
private:

	gyroManager *gyroManagerRun;
	CommandManager *autoCom;
};

START_ROBOT_CLASS(Robot)
