
#ifndef SRC_CALIBRATIONS_H_
#define SRC_CALIBRATIONS_H_

//SHOOTER cals
double shooterSpdP = 0.01;
double shooterSpdI = 0.01;
double shooterSpdD = 0;
double shooterMaxRevPerSec = 88.5;
double shooterEncCntPerRev = .05;

//Near shot is 4 Anthony Feet from corner of boiler and driver wall to robot gear guard
//Med shot is 8 Anthony Feet from corner of boiler and driver wall to robot gear guard
//

double shooterAimIncrement = .05;
double shooterAngNearShot = .3;
double shooterAngFarShot = .4;

double shooterAngAutoNearShot = .45;
double shooterAngAutoMedShot = .6;
double shooterAngAutoFarShot = 1;

double shooterSpdAutoNearShot = 3700;
double shooterSpdAutoMedShot = 3900;
double shooterSpdAutoFarShot = 4600;

double shooterSpdNearShot = 3300; //SPEED is for PID control of shooter speed in RPM
double shooterSpdFarShot = 3600;

//FEEDER cals

//INTAKE cals
double fuelIntakeSpeedForward = 1;
double fuelIntakeSpeedReverse = -1;

//TURN MOTOR PID cals
double turnMotorP = .025;
double turnMotorI = 0;
double turnMotorD = 0;

//DRIVE MOTOR cals
double drvWhlDistPerEncCnt = .094; //InchesPerPulse



#endif /* SRC_CALIBRATIONS_H_ */
