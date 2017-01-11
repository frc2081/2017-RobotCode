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
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		stick1 = new cntl(0);
		stick2 = new cntl(1);

		swerveLib = new swervelib(20, 20);

		LFEnc = new Encoder(0, 1, false);
		RFEnc = new Encoder(2, 3, false);
		LBEnc = new Encoder(4, 5, false);
		RBEnc = new Encoder(6, 7, false);

		LFMotTurn = new VictorSP(0);
		RFMotTurn = new VictorSP(1);
		LBMotTurn = new VictorSP(2);
		RBMotTurn = new VictorSP(3);

		LFMotDrv = new VictorSP(4);
		RFMotDrv = new VictorSP(5);
		LBMotDrv = new VictorSP(6);
		RBMotDrv = new VictorSP(7);

		p = 0.1;
		i = 0.1;
		d = 0.1;
		LFPID = new PIDController(p, i, d, LFEnc, LFMotTurn, period);
		RFPID = new PIDController(p, i, d, RFEnc, RFMotTurn, period);
		LBPID = new PIDController(p, i, d, LBEnc, LBMotTurn, period);
		RBPID = new PIDController(p, i, d, RBEnc, RBMotTurn, period);

	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

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

		LFPID->Enable();
		RFPID->Enable();
		LBPID->Enable();
		RBPID->Enable();

	}

	void TeleopPeriodic() {
		stick1->UpdateCntl();
		stick2->UpdateCntl();

		swerveLib->calcWheelVect(stick1->LX, stick1->LY, stick1->RX);

		RFPID->SetSetpoint(swerveLib->whl->angle1);
		LFPID->SetSetpoint(swerveLib->whl->angle2);
		RBPID->SetSetpoint(swerveLib->whl->angle4);
		LBPID->SetSetpoint(swerveLib->whl->angle3);

		LFMotDrv->Set(swerveLib->whl->speed2);
		RFMotDrv->Set(swerveLib->whl->speed1);
		RBMotDrv->Set(swerveLib->whl->speed4);
		LBMotDrv->Set(swerveLib->whl->speed3);

		printf("%.2f, %.2f, %.2f, %.2f\n", swerveLib->whl->angle1, swerveLib->whl->angle2, swerveLib->whl->angle3, swerveLib->whl->angle4);
		printf("%.2f, %.2f, %.2f, %.2f\n\n", swerveLib->whl->speed1, swerveLib->whl->speed2, swerveLib->whl->speed3, swerveLib->whl->speed4);

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
