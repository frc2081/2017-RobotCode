/*
 * gyroManager.cpp
 *
 *  Created on: Jan 26, 2017
 *      Author: FIRSTUser
 */

#include "gyroManager.h"
gyroManager* gyroManager::_manager = 0;

gyroManager::gyroManager() {

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

void gyroManager::resetGyro() {
	gyroCompass->Reset();
}

void gyroManager::start() {
	if (gyroManager::isRunningCheck()) {
		return;
	}
	keepRunning = true;
	try {
	gyro_thread = thread(&gyroManager::gyroPoll, this);
	} catch (...) {
		keepRunning = false;
		isRunning = false;
	}
}

void gyroManager::gyroPoll() {
	isRunning = true;
	try {
	while (keepRunning) {
		double readValue;
		//Can operate at + or - 300 degrees per second max
		readValue = gyroCompass->GetAngle();

		lockGyroThread.lock();
		lastValue = readValue;
		lockGyroThread.unlock();
	}

	} catch (...) {
		keepRunning = false;
	}
	isRunning = false;
}

void gyroManager::stop() {

	keepRunning = false;
	gyro_thread.join();
}

bool gyroManager::isRunningCheck() {
	return isRunning && keepRunning;
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

