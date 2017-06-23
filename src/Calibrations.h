
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
//Far shot is 12 Anthony Feet from corner of boiler and driver wall to robot gear guard

double shooterAimIncrement = .05;
double shooterAngNearShot = .3;
double shooterAngFarShot = .45;

double shooterAngAutoNearShot = .4;
double shooterAngAutoMedShot = .6;
double shooterAngAutoFarShot = 1;
double shooterAngBinShot = .4;

double shooterSpdAutoNearShot = 3000;
double shooterSpdAutoMedShot = 3900;
double shooterSpdAutoFarShot = 4600;
double shooterSpdAutoBinShot = 3700;

double shooterSpdNearShot = 3000; //SPEED is for PID control of shooter speed in RPM
double shooterSpdFarShot = 3600;

//FEEDER cals

//INTAKE cals
double fuelIntakeSpeedForward = 1;
double fuelIntakeSpeedReverse = -1;

//TURN MOTOR PID cals
double turnMotorP = .04;
double turnMotorI = 0;
double turnMotorD = 0;

//DRIVE MOTOR cals
double drvWhlDistPerEncCnt = .094; //InchesPerPulse
double LFOffset = 1;
double RFOffset = 1;
double LBOffset = 1;
double RBOffset = 1;

double winchDriveFactor = 1;
double winchDriveAngle = 0;

#endif /* SRC_CALIBRATIONS_H_ */
