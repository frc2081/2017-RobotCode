/*
 * CameraFeeds.cpp
 *
 *  Created on: Feb 16, 2015
 *      Author: Mike Anderson
 */

#include <WPILib.h>
#include <CameraFeeds.h>

	CAMERAFEEDS::CAMERAFEEDS(Joystick *newJoy) {
		int imaqError;
		imaqError = IMAQdxOpenCamera(camNameFront, IMAQdxCameraControlModeController, &camFront);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
		}
		imaqError = IMAQdxOpenCamera(camNameBack, IMAQdxCameraControlModeController, &camBack);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
		}
		curCam = camBack;
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		server = CameraServer::GetInstance();
		server->SetQuality(imgQuality);
		contrlr = newJoy;
		kBtPrev = false;
		kBtCurr = false;
		currCamNum  = 1;
		delay = true;

		cameraDelay = new Timer();
	}

	CAMERAFEEDS::~CAMERAFEEDS() {

	}
	void CAMERAFEEDS::init() {
		changeCam (camBack);
		cameraThread = new Task("Camera Task", &CAMERAFEEDS::run, this);
	}

	void CAMERAFEEDS::end() {
		IMAQdxStopAcquisition(curCam);
	}

	void CAMERAFEEDS::changeCam(int newId) {
		int imaqError;
		IMAQdxStopAcquisition(curCam);
		imaqError = IMAQdxConfigureGrab(newId);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError) + "\n");
		}
		IMAQdxStartAcquisition(newId);
		curCam = newId;
	}

	void CAMERAFEEDS::updateCam() {
		int imaqError;
		imaqError = IMAQdxGrab(curCam, frame, true, NULL);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
		}
		server->SetImage(frame);

	}

	void CAMERAFEEDS::run()
	{
		printf("Camera Running");
		while(true)
		{
			Wait(.1);
			kBtPrev = kBtCurr;
			kBtCurr = contrlr->GetRawButton(CAMERAFEEDS::kBtCamToggle);

			if(kBtCurr == true && kBtPrev == false)
			{
				if (currCamNum == 1) {
					changeCam(camFront);
					currCamNum = 0;
				}
				else if (currCamNum == 0) {
					changeCam(camBack);
					currCamNum = 1;
				}
			}
			updateCam();
			printf("*");
		}
	}

