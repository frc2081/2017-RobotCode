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

	//Infinite ducks



private:

	//Constructor
	gyroManager();

	//Destructor
	virtual ~gyroManager();

	static thread gyro_thread;

	static bool keepRunning;

	static bool isRunning;

	static double lastValue;

	static mutex lockGyroThread;

	static ADXRS450_Gyro *gyroCompass;

	static void gyroPoll();

	static gyroManager *manager;
};

#endif /* SRC_GYROMANAGER_H_ */
