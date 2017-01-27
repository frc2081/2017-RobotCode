/*
 * gyroManager.cpp
 *
 *  Created on: Jan 26, 2017
 *      Author: FIRSTUser
 */

#include "gyroManager.h"
gyroManager* gyroManager::_manager = 0;

gyroManager::gyroManager() {
	// TODO Auto-generated constructor stub

	keepRunning = false;

	isRunning = false;

	lastValue = 0;

	gyroCompass = new ADXRS450_Gyro();

	gyroCompass->Calibrate();

}

gyroManager *gyroManager::Get() {
	if (_manager == 0) {
		_manager = new gyroManager();
	}

	return _manager;
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
		double readValue;
		readValue = gyroCompass->GetAngle();

		lockGyroThread.lock();
		lastValue = readValue;
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
	delete gyroCompass;
	stop();
}

