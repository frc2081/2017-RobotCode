/*
 * gyroManager.cpp
 *
 *  Created on: Jan 26, 2017
 *      Author: FIRSTUser
 */

#include "gyroManager.h"
gyroManager *gyroManager::manager = 0;
gyroManager::gyroManager() {
	// TODO Auto-generated constructor stub

	keepRunning = false;

	isRunning = false;

	lastValue = 0;

	//gyroCompass = new ADXRS450_Gyro();

	//gyroCompass->Calibrate();

}

gyroManager *gyroManager::Get() {
	if (manager == 0) {
		manager = new gyroManager();
	}
	return manager;
}

void gyroManager::start() {
	if (keepRunning && isRunning) {
		return;
	}

	gyro_thread = thread(&gyroManager::gyroPoll, this);
}

void gyroManager::gyroPoll() {
	isRunning = true;
	while (keepRunning) {
		lockGyroThread.lock();
		//lastValue = gyroCompass->GetAngle();
		lockGyroThread.unlock();
	}
	isRunning = false;
}

void gyroManager::stop() {

	keepRunning = false;

}

double gyroManager::getLastValue() {

	double currValue;
	lockGyroThread.lock();
	currValue = lastValue;
	lockGyroThread.unlock();
	return currValue;

}

gyroManager::~gyroManager() {
	// TODO Auto-generated destructor stub

	//delete gyroCompass;

	stop();
}

