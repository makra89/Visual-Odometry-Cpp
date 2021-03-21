/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include<Vocpp_Master/VocppMaster.h>
#include<Vocpp_Interface/Frame.h>
#include<Vocpp_Interface/Tracing.h>
#include<Vocpp_Interface/DeltaCameraPose.h>
#include<Vocpp_Interface/CameraPose.h>
#include<Vocpp_Calibration/MonoCameraCalibration.h>

using namespace cv;
using namespace VOCPP;

class StdOutTracer : public Tracer
{
public:
    virtual void receiveTrace(const TraceLevel::Enum& in_traceLevel, const char* const in_msg)
    {
        std::cout << in_msg << std::endl;
    }
};

int32_t main(int32_t argc, char** argv)
{
    if (argc != 2)
   {
        printf("usage: main.exe <Image_Path> (e.g. ./data/*.jpg) \n");
        return -1;
    }

    // Construct path
    String path(argv[1]);
    std::vector<String> imageNames;
    glob(path, imageNames);

    // Activate tracing on Debug level
    SetTraceLevel(TraceLevel::TL_DEBUG);
    StdOutTracer tracer;
    RegisterTracer(&tracer);

    // Instantiate master
    VocppMaster voMaster;
    voMaster.ActivateDebugOutput();

    // Construct MonoCameraCalibration
    // Hardcoded for KITTI data set sequence 0, left image
    const double focLength = 7.18856e+02;
    const double camCenterX = 6.071928000000e+02;
    const double camCenterY = 1.852157000000e+02;
    const double skew = 0.0;
    VOCPP::Calibration::MonoCameraCalibration monoCalib(focLength, camCenterX, camCenterY, skew);

    // Load calibration, we know it is valid
    (void)voMaster.LoadCalibration(monoCalib);

    int32_t frameId = 0U;

    cv::Mat1d currentPose = cv::Mat1d::eye(3, 3);
    cv::Mat1d currentCamCenter = cv::Mat1d::zeros(3, 1);
    
    for (uint32_t imageIdx = 0U; imageIdx < imageNames.size(); imageIdx++)
    {
        // Convert the image to grayscale
        cv::Mat grayScaleImg;
        cv::cvtColor(imread(imageNames[imageIdx], 1), grayScaleImg, COLOR_BGR2GRAY);
        
        // Feed frame to Master
        Frame frame(grayScaleImg.ptr<uint8_t>(0), grayScaleImg.cols, grayScaleImg.rows, frameId);
        voMaster.FeedNextFrame(frame);
        
        DeltaCameraPose delta = voMaster.GetLastDeltaPose();
        CameraPose pose = voMaster.GetLastPose();

        frameId++;
    }
   
    return 0;
}