/*
 *  cntl.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: Lunar Dust
 */
#include "cntl.h"

//For practicality
enum btnNums {
	_bA,
	_bB,
	_bX,
	_bY,
	_bLB,
	_bRB,
	_bBack,
	_bStart,
	_bLS,
	_bRS
};

btn::btn(int button, frc::Joystick **s) {
	M_raw = new Btn_t(*s, button);
	Hld = false;
	State = false;
	RE = false;
}

void btn::Update() {
	Hld = State;
	State = M_raw->Grab();
	RE = false;
	if(Hld != State && !Hld) {
		RE = true;
	}
}

cntl::cntl(int port, double deadzone) {
	M_stick = new frc::Joystick(port);
	RX = 0.0;
	RY = 0.0;
	LX = 0.0;
	LY = 0.0;
	RTrig = 0.0;
	LTrig = 0.0;

	M_deadzone = deadzone;

	bA = new btn(1, &M_stick);
	bB = new btn(2, &M_stick);
	bX = new btn(3, &M_stick);
	bY = new btn(4, &M_stick);
	bLB = new btn(5, &M_stick);
	bRB = new btn(6, &M_stick);
	bBack = new btn(7, &M_stick);
	bStart = new btn(8, &M_stick);
	bLS = new btn(9, &M_stick);
	bRS = new btn(10, &M_stick);
}

void applyDeadzone(double *var, double input, double zone) {
	double output = fabs(input) - zone;
	output /= (1 - zone);
	if(output < 0) {
		output = 0;
	}
	if(input < 0) {
		output *= -1;
	}
	if((output + 0.0) == 0.0) {
		output = 0.0;
	}
	*var = output;
}

void cntl::UpdateCntl() {
	applyDeadzone(&this->LX, M_stick->GetX(), M_deadzone);
	applyDeadzone(&this->LY, M_stick->GetY(), M_deadzone);
	applyDeadzone(&this->RX, M_stick->GetRawAxis(4), M_deadzone);
	applyDeadzone(&this->RY, M_stick->GetRawAxis(5), M_deadzone);
	applyDeadzone(&this->RTrig, M_stick->GetRawAxis(3), M_deadzone);
	applyDeadzone(&this->LTrig, M_stick->GetRawAxis(2), M_deadzone);
	RY *= -1;
	LY *= -1;

	bA->Update();
	bB->Update();
	bX->Update();
	bY->Update();
	bLB->Update();
	bRB->Update();
	bBack->Update();
	bStart->Update();
	bLS->Update();
	bRS->Update();
}

