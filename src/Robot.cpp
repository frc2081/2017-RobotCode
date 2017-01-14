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

	double deadband(double input) {
		double output;
		output = fabs(input)-.2;
		output /= .8;
		if (output < 0) {
			output = 0;
		}
		if (input < 0) {
			output *= -1;
		}
		return output;
	}
	void RobotInit() {
		//chooser.AddDefault(autoNameDefault, autoNameDefault);
		//chooser.AddObject(autoNameCustom, autoNameCustom);
		//frc::SmartDashboard::PutData("Auto Modes", &chooser);

		//Instanciate the joysticks according to the controller class
		stick1 = new cntl(0);
		stick2 = new cntl(1);

		//Instanciate the swerve calculations library
		swerveLib = new swervelib(20, 20);

		gyroCompass = new ADXRS450_Gyro();

		//Instanciate the encoders
		LFEnc = new AnalogPotentiometer(1, false, 0);
		RFEnc = new AnalogPotentiometer(0, false, 0);
		LBEnc = new AnalogPotentiometer(3, false, 0);
		RBEnc = new AnalogPotentiometer(2, false, 0);

		//Instanciate the motors based on where they are plugged in
		LFMotTurn = new Talon(8);
		RFMotTurn = new Talon(7);
		LBMotTurn = new Talon(2);
		RBMotTurn = new Talon(1);

		LFMotDrv = new Talon(4);
		RFMotDrv = new Talon(5);
		LBMotDrv = new Talon(6);
		RBMotDrv = new Talon(3);


		//Instanciate the PID controllers to their proper values
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
		gyroCompass->Calibrate();

		//Get the numbers from the smartdashboard for live PID tuning
		SmartDashboard::GetNumber("P: ", 0);
		SmartDashboard::GetNumber("I: ", 0);
		SmartDashboard::GetNumber("D: ", 0);

	}

	void TeleopPeriodic() {
		//Update the joystick values
		stick1->UpdateCntl();
		stick2->UpdateCntl();

		stick1->LX = this->deadband(stick1->LX);
		stick1->LY = this->deadband(stick1->LY);
		stick1->RX = this->deadband(stick1->RX);
		stick1->RY = this->deadband(stick1->RY);
		facing = gyroCompass->GetAngle();

		//Get the numbers from the smartdashboard for live PID tuning
		p = SmartDashboard::GetNumber("P: ", 0);
		i = SmartDashboard::GetNumber("I: ", 0);
		d = SmartDashboard::GetNumber("D: ", 0);

		//Set the PID values for live tuning
		RFPID->SetPID(p, i, d);
		LFPID->SetPID(p, i, d);
		RBPID->SetPID(p, i, d);
		LBPID->SetPID(p, i, d);

		//Calculate the proper values for the swerve drive motion
		swerveLib->calcWheelVect(stick1->LX, stick1->LY, stick1->RX);

		//Set the PID controllers to angle the wheels properly
		RFPID->SetSetpoint(swerveLib->whl->angle1);
		LFPID->SetSetpoint(swerveLib->whl->angle2);
		RBPID->SetSetpoint(swerveLib->whl->angle4);
		LBPID->SetSetpoint(swerveLib->whl->angle3);

		//Set the wheel speed based on what the calculations from the swervelib
		LFMotDrv->Set(swerveLib->whl->speed2);
		RFMotDrv->Set(swerveLib->whl->speed1);
		RBMotDrv->Set(swerveLib->whl->speed4);
		LBMotDrv->Set(swerveLib->whl->speed3);

		printf("%.2f, %.2f, %.2f, %.2f\n", swerveLib->whl->angle1, swerveLib->whl->angle2, swerveLib->whl->angle3, swerveLib->whl->angle4);
		printf("%.2f, %.2f, %.2f, %.2f\n\n", swerveLib->whl->speed1, swerveLib->whl->speed2, swerveLib->whl->speed3, swerveLib->whl->speed4);
		printf("%.2f, %.2f, %.2f\n\n", stick1->LX, stick1->LY, stick1->RX);


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
