
#ifndef SRC_CALIBRATIONS_H_
#define SRC_CALIBRATIONS_H_

//SHOOTER cals
double shooterSpdP = 0.01;
double shooterSpdI = 0.01;
double shooterSpdD = 0;
double shooterMaxRevPerSec = 88.5;
double shooterEncCntPerRev = .05;

double shooterAimIncrement = .05;
double shooterAngNearShot = .3;
double shooterAngFarShot = .4;
double shooterAngAutoNearShot = .3;
double shooterAngAutoMedShot = .8;
double shooterAngAutoFarShot = 1;

double shooterPwrAutoNearShot = .7; //POWER is for directly commanding the shooter motor
double shooterPwrAutoMedShot = .9; //POWER is for directly commanding the shooter motor
double shooterPwrAutoFarShot = 1;

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

//VISION cals
//Cals are for a 320x240 image
int liftImageWidth = 320;
int liftImageHeight = 240;
double liftMaxRatio = 7;
double liftMinRatio = 1.5;
int liftCenterMaxYDiff = 10;
int liftCenterMaxYPos = 200;
int liftCenterMaxXDist = 300;
int liftCenterMinXDiff = 50;
bool liftTargetAcquired = false;

#endif /* SRC_CALIBRATIONS_H_ */
