#include "Camera.h"

    void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(320, 240);
        camera.SetExposureManual(-10);
        camera.SetFPS(15);
        //cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
        //cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
        //cv::Mat source;
       // while(true) {
          //  cvSink.GrabFrame(source);
       // //    outputStreamStd.PutFrame(source);
       //}
    }


