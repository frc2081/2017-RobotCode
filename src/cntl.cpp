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

btn::btn(int button, Joystick **s) {
	raw = new JoystickButton(*s, button);
	Hld = false;
	State = false;
	RE = false;
}

void btn::Update() {
	this->Hld = this->State;
	this->State = this->raw->Grab();
	this->RE = false;
	if(this->Hld != this->State && !this->Hld) {
		this->RE = true;
	}
}

cntl::cntl(int port) {
	stick = new Joystick(port);
	this->RX = 0.0;
	this->RY = 0.0;
	this->LX = 0.0;
	this->LY = 0.0;
	this->RTrig = 0.0;
	this->LTrig = 0.0;

	this->bA = new btn(1, &stick);
	this->bB = new btn(2, &stick);
	this->bX = new btn(3, &stick);
	this->bY = new btn(4, &stick);
	this->bLB = new btn(5, &stick);
	this->bRB = new btn(6, &stick);
	this->bBack = new btn(7, &stick);
	this->bStart = new btn(8, &stick);
	this->bLS = new btn(9, &stick);
	this->bRS = new btn(10, &stick);
}

void cntl::UpdateCntl() {
	this->LX = this->stick->GetX();
	this->LY = this->stick->GetY();
	this->RX = this->stick->GetRawAxis(4);
	this->RY = this->stick->GetRawAxis(5);
	this->RTrig = this->stick->GetRawAxis(3);
	this->LTrig = this->stick->GetRawAxis(2);
	this->RY *= -1;
	this->LY *= -1;
	this->bA->Update();
	this->bB->Update();
	this->bX->Update();
	this->bY->Update();
	this->bLB->Update();
	this->bRB->Update();
	this->bBack->Update();
	this->bStart->Update();
	this->bLS->Update();
	this->bRS->Update();
}

