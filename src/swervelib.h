/* Copyright 2016 Charles Young */

#ifndef SWERVELIB_H_
#define SWERVELIB_H_

#include <cmath>

const double PI = 3.14159265;
class swervelib {
	private:
		double MAX_WS;
		double A, B, C, D, R, width, length;
	public:
		double W1[2], W2[2], W3[2], W4[2];

		swervelib(double, double);

		virtual ~swervelib() = default;

		//Arguments: x, y, rotation
		void calcWheelVect(double, double, double);
};
#endif //SWERVELIB_H_
