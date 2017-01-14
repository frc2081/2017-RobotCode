#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "Robot.h"

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() {
		//Instantiate the joysticks according to the controller class
		stick1 = new cntl(0, 0.2);
		//stick2 = new cntl(1, 0.2);

		//Instantiate the swerve calculations library
		swerveLib = new swervelib(20, 20);

		gyroCompass = new ADXRS450_Gyro();

		//Instantiate the encoders
		LFEnc = new AnalogPotentiometer(1, false, 0);
		RFEnc = new AnalogPotentiometer(0, false, 0);
		LBEnc = new AnalogPotentiometer(3, false, 0);
		RBEnc = new AnalogPotentiometer(2, false, 0);

		//Instantiate the motors based on where they are plugged in
		LFMotTurn = new Talon(8);
		RFMotTurn = new Talon(7);
		LBMotTurn = new Talon(2);
		RBMotTurn = new Talon(1);

		LFMotDrv = new Talon(4);
		RFMotDrv = new Talon(5);
		LBMotDrv = new Talon(6);
		RBMotDrv = new Talon(3);


		//Instantiate the PID controllers to their proper values
		p = 0;
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

		SmartDashboard::init();
	}

	void AutonomousInit() override {
	}

	void AutonomousPeriodic() {
	}

	void TeleopInit() {
		gyroCompass->Calibrate();

		//Get the numbers from the smartdashboard for live PID tuning
		/*SmartDashboard::GetNumber("P: ", 0);
		SmartDashboard::GetNumber("I: ", 0);
		SmartDashboard::GetNumber("D: ", 0);*/

	}

	void TeleopPeriodic() {
		//Update the joystick values
		stick1->UpdateCntl();
		stick2->UpdateCntl();

		//Calculate the proper values for the swerve drive motion
		swerveLib->calcWheelVect(stick1->LX, stick1->LY, stick1->RX);

		//Set the PID controllers to angle the wheels properly
		//RFPID->SetSetpoint(swerveLib->whl->angle1/72);
		//LFPID->SetSetpoint(swerveLib->whl->angle2/72);
		//RBPID->SetSetpoint(swerveLib->whl->angle4/72);
		//LBPID->SetSetpoint(swerveLib->whl->angle3/72);

		LFMotTurn->Set(swerveLib->W2[a]/360);
		RFMotTurn->Set(swerveLib->W1[a]/360);
		LBMotTurn->Set(swerveLib->W3[a]/360);
		RBMotTurn->Set(swerveLib->W4[a]/360);


		//Set the wheel speed based on what the calculations from the swervelib
		LFMotDrv->Set(swerveLib->W2[s]/11.5);
		RFMotDrv->Set(swerveLib->W1[s]/11.5);
		LBMotDrv->Set(swerveLib->W3[s]/11.5);
		RBMotDrv->Set(swerveLib->W4[s]/11.5);

		SmartDashboard::PutNumber("RX", stick1->RX);
		SmartDashboard::PutNumber("RY", stick1->RY);
		SmartDashboard::PutNumber("LX", stick1->LX);
		SmartDashboard::PutNumber("LY", stick1->LY);

		SmartDashboard::PutNumber("Wheel 1 speed", swerveLib->W1[s]);
		SmartDashboard::PutNumber("Wheel 2 speed", swerveLib->W2[s]);
		SmartDashboard::PutNumber("Wheel 3 speed", swerveLib->W3[s]);
		SmartDashboard::PutNumber("Wheel 4 speed", swerveLib->W4[s]);

		SmartDashboard::PutNumber("Wheel 1 angle", swerveLib->W1[a]);
		SmartDashboard::PutNumber("Wheel 2 angle", swerveLib->W2[a]);
		SmartDashboard::PutNumber("Wheel 3 angle", swerveLib->W3[a]);
		SmartDashboard::PutNumber("Wheel 4 angle", swerveLib->W4[a]);
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
