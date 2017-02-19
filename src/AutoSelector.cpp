#include <AutoSelector.h>

AutoSelector::AutoSelector(int Channel) {

	autoDials = new AnalogInput(Channel);
}

robotAction AutoSelector::getAction() {

	double voltage = autoDials->GetVoltage();

	if (voltage >= rangeOneLow && voltage <= rangeOneHigh) {
		action = CROSS_MIDLINE;
	} else if (voltage >= rangeTwoLow && voltage <= rangeTwoHigh) {
		action = GEAR_ONLY;
	} else if (voltage >= rangeThreeLow && voltage <= rangeThreeHigh) {
		action = SHOOT_ONLY;
	} else if (voltage >= rangeFourLow && voltage <= rangeFourHigh) {
		action = GEAR_AND_SHOOT;
	} else if (voltage >= rangeFiveLow && voltage <= rangeFiveHigh) {
		action = SHOOT_ONLY_BIN;
	} else action = NO_AUTO;

	return action;
}

robotStation AutoSelector::getFieldPosition()
{
	double voltage = autoDials->GetVoltage();

	if (voltage >= rangeOneLow && voltage <= rangeOneHigh) {
			fieldPosition = ONE;
		} else if (voltage >= rangeTwoLow && voltage <= rangeTwoHigh) {
			fieldPosition = TWO;
		} else if (voltage >= rangeThreeLow && voltage <= rangeThreeHigh) {
			fieldPosition = THREE;
		} else fieldPosition = ONE;
	return fieldPosition;
}
