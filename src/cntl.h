/*
 * 	WPILib Controller library
 *
 * 	Organization: FRC2081
 *  Author: Lunar Dust
 *
 *  Do not redistribute this library without the permission of the author.
 */

#ifndef CNTL_H_
#define CNTL_H_

#include "WPILib.h"

typedef JoystickButton Btn_t;

class btn
{
	private:
		//JoystickButton object
		Btn_t *raw;

	public:
		//The current state of the given button
		bool State;
		//True if the current state is true and the last state is false
		bool RE;
		//The state of the button on the previous loop
		bool Hld;

		//Called for all buttons on a controller by calling UpdateCntl()
		void Update();

		//Button constructor
		btn(int, Joystick**);
		virtual ~btn() = default;
};

class cntl
{
	private:
		//Joystick object
		Joystick *stick;

	public:
		//Returns controller object, takes controller number as argument
		cntl(int);

		float LX;
		float LY;
		float RX;
		float RY;
		float RTrig;
		float LTrig;

		//Should be called for each controller at the beginning of each teleop loop
		void UpdateCntl();

		//Button objects of the controller
		btn *bA;
		btn *bB;
		btn *bX;
		btn *bY;
		btn *bLB;
		btn *bRB;
		btn *bBack;
		btn *bStart;
		btn *bLS;
		btn *bRS;
};

#endif /* CNTL_H_ */
