#include <AutoSelector.h>

AutoSelector::AutoSelector(int Channel) {

	autoDials = new AnalogInput(Channel);
	if (autoDials->GetVoltage() >= rangeOneLow && autoDials->GetVoltage() <= rangeOneHigh) {
		action = CROSS_MIDLINE;
	} else if (autoDials->GetVoltage() >= rangeTwoLow && autoDials->GetVoltage() <= rangeTwoHigh) {
		action = GEAR_ONLY;
	} else if (autoDials->GetVoltage() >= rangeThreeLow && autoDials->GetVoltage() <= rangeThreeHigh) {
		action = SHOOT_ONLY;
	} else if (autoDials->GetVoltage() >= rangeFourLow && autoDials->GetVoltage() <= rangeFourHigh) {
		action = GEAR_AND_SHOOT;
	} else if (autoDials->GetVoltage() >= rangeFiveLow && autoDials->GetVoltage() <= rangeFiveHigh) {
		action = SHOOT_ONLY_BIN;
	}

}

robotAction AutoSelector::getSelection() {
	return action;
}
