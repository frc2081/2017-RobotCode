#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "Robot.h"

Talon *LFMotDrv;
Talon *LBMotDrv;
Talon *RFMotDrv;
Talon *RBMotDrv;
Talon *LFMotTurn;
Talon *LBMotTurn;
Talon *RFMotTurn;
Talon *RBMotTurn;

AnalogPotentiometer *LFEnc;
AnalogPotentiometer *LBEnc;
AnalogPotentiometer *RFEnc;
AnalogPotentiometer *RBEnc;

Encoder *LFEncDrv;
Encoder *RFEncDrv;
Encoder *LBEncDrv;
Encoder *RBEncDrv;

cntl *cntl1;
cntl *cntl2;

swervelib *swerveLib;

ADXRS450_Gyro *gyroCompass;

PIDController *LFPID;
PIDController *LBPID;
PIDController *RFPID;
PIDController *RBPID;

float p, i, d;
float currentAngle;
float currentFacing;
float comAng, comMag;
double currAng1, currAng2, currAng3, currAng4;
double preAng1, preAng2, preAng3, preAng4;
class Robot: public frc::IterativeRobot {
public:
	void RobotInit() {
		//chooser.AddDefault(autoNameDefault, autoNameDefault);
		//chooser.AddObject(autoNameCustom, autoNameCustom);
		//frc::SmartDashboard::PutData("Auto Modes", &chooser);

		//Instantiate the joysticks according to the controller class
		cntl1 = new cntl(0, .2);
		cntl2 = new cntl(1, .2);

		//Instantiate the swerve calculations library
		swerveLib = new swervelib(27, 23.5);

		//gyroCompass = new ADXRS450_Gyro();

		//Instantiate the encoders
		LFEnc = new AnalogPotentiometer(1, 360, 0);
		RFEnc = new AnalogPotentiometer(0, 360, 0);
		LBEnc = new AnalogPotentiometer(3, 360, 0);
		RBEnc = new AnalogPotentiometer(2, 360, 0);

		LFEncDrv = new Encoder(6, 7, false);
		RFEncDrv = new Encoder(4, 5, false);
		LBEncDrv = new Encoder (0, 1, false);
		RBEncDrv = new Encoder (2, 3, false);

		//Instantiate the motors based on where they are plugged in
		LFMotTurn = new Talon(8);
		RFMotTurn = new Talon(7);
		LBMotTurn = new Talon(2);
		RBMotTurn = new Talon(1);

		LFMotDrv = new Talon(6);
		RFMotDrv = new Talon(5);
		LBMotDrv = new Talon(4);
		RBMotDrv = new Talon(3);


		//Instantiate the PID controllers to their proper values
		p = .025;
		i = 0;
		d = 0;
		LFPID = new PIDController(p, i, d, LFEnc, LFMotTurn, period);
		RFPID = new PIDController(p, i, d, RFEnc, RFMotTurn, period);
		LBPID = new PIDController(p, i, d, LBEnc, LBMotTurn, period);
		RBPID = new PIDController(p, i, d, RBEnc, RBMotTurn, period);

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

		//gyroCompass->Calibrate();

		LFEncDrv->Reset();
		RFEncDrv->Reset();
		LBEncDrv->Reset();
		RBEncDrv->Reset();
	}

	void AutonomousInit() override {
		//autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		//std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {
	}

	void TeleopPeriodic() {

		//Update the joystick values
		cntl1->UpdateCntl();
		cntl2->UpdateCntl();


		//Set the PID values for live tuning
		RFPID->SetPID(p, i, d);
		LFPID->SetPID(p, i, d);
		RBPID->SetPID(p, i, d);
		LBPID->SetPID(p, i, d);

		SmartDashboard::PutNumber("LFEnc: ", LFEnc->Get());
		SmartDashboard::PutNumber("RFEnc: ", RFEnc->Get());
		SmartDashboard::PutNumber("LBEnc: ", LBEnc->Get());
		SmartDashboard::PutNumber("RBEnc: ", RBEnc->Get());

		//Get current gyro heading
		/*currentFacing = fabs(gyroCompass->GetAngle());

		if (currentFacing > 360) {
			currentFacing = (int)currentFacing % 360;
		}
*/

		currAng1 = swerveLib->whl->angle1;
		currAng2 = swerveLib->whl->angle2;
		currAng3 = swerveLib->whl->angle3;
		currAng4 = swerveLib->whl->angle4;

		comAng = (atan2(-cntl1->LX, cntl1->LY) * 180/PI);// + currentFacing;
		comMag = sqrt(pow(cntl1->LX, 2) + pow(cntl1->LY, 2));

		//Calculate the proper values for the swerve drive motion
		if (cntl1->LX != 0 || cntl1->LY != 0 || cntl1->RX != 0) {
			swerveLib->calcWheelVect(comMag, comAng, cntl1->RX);
		} else {
			swerveLib->whl->speed1 = 0;
			swerveLib->whl->speed2 = 0;
			swerveLib->whl->speed3 = 0;
			swerveLib->whl->speed4 = 0;
		}

		if (fabs(swerveLib->whl->angle1 - currAng1) > 90 && fabs(swerveLib->whl->angle1 - currAng1) < 270) {
			swerveLib->whl->angle1 = ((int)swerveLib->whl->angle1 + 180) % 360;
			swerveLib->whl->speed1 *= -1;
		}

		if (fabs(swerveLib->whl->angle2 - currAng2) > 90 && fabs(swerveLib->whl->angle2 - currAng2) < 270) {
			swerveLib->whl->angle2 = ((int)swerveLib->whl->angle2 + 180) % 360;
			swerveLib->whl->speed2 *= -1;
		}

		if (fabs(swerveLib->whl->angle3 - currAng3) > 90 && fabs(swerveLib->whl->angle3 - currAng3) < 270) {
			swerveLib->whl->angle3 = ((int)swerveLib->whl->angle3 + 180) % 360;
			swerveLib->whl->speed3 *= -1;
		}

		if (fabs(swerveLib->whl->angle4 - currAng4) > 90 && fabs(swerveLib->whl->angle4 - currAng4) < 270) {
			swerveLib->whl->angle4 = ((int)swerveLib->whl->angle4 + 180) % 360;
			swerveLib->whl->speed4 *= -1;
		}

		//Set the PID controllers to angle the wheels properly
		RFPID->SetSetpoint(swerveLib->whl->angle1);
		LFPID->SetSetpoint(swerveLib->whl->angle2);
		RBPID->SetSetpoint(swerveLib->whl->angle4);
		LBPID->SetSetpoint(swerveLib->whl->angle3);

		//If the commanded speeds are 0, keep the wheels in the position they were in  before
		/*
		if (fabs(cntl1->LX) == 0 && fabs(cntl1->LY) == 0 && fabs(cntl1->RX) == 0) {
			RFPID->SetSetpoint(preAng1);
			LFPID->SetSetpoint(preAng2);
			RBPID->SetSetpoint(preAng4);
			LBPID->SetSetpoint(preAng3);
		}
		*/

		//Set the wheel speed based on what the calculations from the swervelib
		LFMotDrv->Set(swerveLib->whl->speed2 * -1);
		RFMotDrv->Set(swerveLib->whl->speed1 * -1);
		RBMotDrv->Set(swerveLib->whl->speed4 * -1);
		LBMotDrv->Set(swerveLib->whl->speed3 * -1);

		//Debug print statements
		SmartDashboard::PutNumber("LF: ", LFEncDrv->Get());
		SmartDashboard::PutNumber("RF: ", RFEncDrv->Get());
		SmartDashboard::PutNumber("LB: ", LBEncDrv->Get());
		SmartDashboard::PutNumber("RB: ", RBEncDrv->Get());
		printf("%.2f, %.2f, %.2f, %.2f\n", swerveLib->whl->angle1, swerveLib->whl->angle2, swerveLib->whl->angle3, swerveLib->whl->angle4);
		printf("%.2f, %.2f, %.2f, %.2f\n\n", swerveLib->whl->speed1, swerveLib->whl->speed2, swerveLib->whl->speed3, swerveLib->whl->speed4);
		printf("%.2f, %.2f, %.2f\n\n", cntl1->LX, cntl1->LY, cntl1->RX);
		//printf("%.5f, %.5f\n\n", gyroCompass->GetAngle(), currentFacing);

	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
