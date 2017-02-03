/*
 * gyroManager.h
 *
 *  Created on: Jan 26, 2017
 *      Author: FIRSTUser
 */

#ifndef SRC_GYROMANAGER_H_
#define SRC_GYROMANAGER_H_
#include <thread>
#include <mutex>
#include <WPILib.h>
using namespace std;


class gyroManager {
public:

	static gyroManager *Get();

	void start();

	void stop();

	double getLastValue();

	bool isRunningCheck();

	void resetGyro();

	//Infinite ducks



private:

	//Constructor
	gyroManager();

	//Destructor
	virtual ~gyroManager();

	thread gyro_thread;

	bool keepRunning;

	bool isRunning;

	double lastValue;

	mutex lockGyroThread;

	ADXRS450_Gyro *gyroCompass;

	void gyroPoll();

	static gyroManager *_manager;
};

#endif /* SRC_GYROMANAGER_H_ */
