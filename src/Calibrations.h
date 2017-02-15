
#ifndef SRC_CALIBRATIONS_H_
#define SRC_CALIBRATIONS_H_

//SHOOTER cals
double shooterSpdP = 0.001;
double shooterSpdI = 0.01;
double shooterSpdD = 0;
double shooterMaxRevPerSec = 88.5;
double shooterEncCntPerRev = .05;

double shooterAimIncrement = .05;
double shooterAngNearShot = .2;
double shooterAngFarShot = .8;
double shooterPwrNearShot = .7; //POWER is for directly commanding the shooter motor
double shooterPwrFarShot = 1;
double shooterSpdNearShot = 50; //SPEED is for PID control of shooter speed
double shooterSpdFarShot = shooterMaxRevPerSec;

//FEEDER cals

//INTAKE cals
double fuelIntakeSpeedForward = 1;
double fuelIntakeSpeedReverse = -1;

//TURN MOTOR PID cals
double turnMotorP = .025;
double turnMotorI = 0;
double turnMotorD = 0;

//DRIVE MOTOR cals
double drvWhlDistPerEncCnt = .1904545454545;

//VISION cals
//Cals are for a 320x240 image
int frameWidth = 320;
int frameHeight = 240;
double liftMaxRatio = 7;
double liftMinRatio = 1.5;
int liftYCenterMaxDiff = 10;
int liftYCenterMaxPos = 120;
int liftXCenterMaxDist = 300;
int liftXCenterMinDiff = 50;
bool liftTargetAcquired = false;

#endif /* SRC_CALIBRATIONS_H_ */
