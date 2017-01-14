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
		Btn_t *M_raw;

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
		Joystick *M_stick;

		double M_deadzone;

	public:
		//Returns controller object, takes controller number and deadzone(0.01 to 1) as argument
		cntl(int, double);

		double LX;
		double LY;
		double RX;
		double RY;
		double RTrig;
		double LTrig;

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
