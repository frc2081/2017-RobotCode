#include "wpilib.h"
#include "autoCommands/CommandIO.h"

class AutoSelector{
	public:
		AutoSelector(int Channel);
		robotAction getSelection();
	
	private:
		double rangeDiagLow = 0.5;
		double rangeDiagHigh = 4.5;
		double rangeOneLow = 0.5;
		double rangeOneHigh = .9125;
		double rangeTwoLow = 1.0125;
		double rangeTwoHigh = 1.425;
		double rangeThreeLow = 1.525;
		double rangeThreeHigh = 1.9375;
		double rangeFourLow = 2.0375;
		double rangeFourHigh = 2.45;
		double rangeFiveLow = 2.55;
		double rangeFiveHigh = 2.96;
		double rangeSixLow = 3.0625;
		double rangeSixHigh = 3.475;
		double rangeSevenLow = 3.575;
		double rangeSevenHigh = 3.98;
		double rangeEightLow = 4.0875;
		double rangeEightHigh = 4.5;
		robotAction action;
		AnalogInput *autoDials;
};
