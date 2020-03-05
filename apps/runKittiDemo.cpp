/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include<Vocpp_Master/Master.h>
#include<Vocpp_Interface/Frame.h>
#include<Vocpp_Interface/DeltaCameraPose.h>
#include<Vocpp_Interface/CameraPose.h>
#include<Vocpp_Calibration/MonoCameraCalibration.h>

using namespace cv;
using namespace VOCPP;

int main(int argc, char** argv)
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

    // Instantiate master
    Master::Master voMaster;

    // Construct MonoCameraCalibration
    // Hardcoded for KITTI data set sequence 0, left image
    const float focLength = 7.18856e+02F;
    const float camCenterX = 6.071928000000e+02F;
    const float camCenterY = 1.852157000000e+02F;
    const float skew = 0.0F;
    VOCPP::Calibration::MonoCameraCalibration monoCalib(focLength, camCenterX, camCenterY, skew);

    // Load calibration, we know it is valid
    (void)voMaster.LoadCalibration(monoCalib);

    int frameId = 0U;

    cv::Mat1f currentPose = cv::Mat1f::eye(3, 3);
    cv::Mat1f currentCamCenter = cv::Mat1f::zeros(3, 1);
    
    for (auto imageName : imageNames)
    {
        // Convert the image to grayscale CV_32F, here from CV_8U to CV_32F
        cv::Mat grayScaleImg;
        cv::cvtColor(imread(imageName, 1), grayScaleImg, COLOR_BGR2GRAY);
        grayScaleImg.convertTo(grayScaleImg, CV_32F, 1.0 / 255.0);
        
        // Feed frame to Master
        const float* buffer = grayScaleImg.ptr<float>(0);
        Frame frame(grayScaleImg.ptr<float>(0), grayScaleImg.cols, grayScaleImg.rows, frameId);
        voMaster.FeedNextFrame(frame);
        
        DeltaCameraPose delta = voMaster.GetLastDeltaPose();
        CameraPose pose = voMaster.GetLastPose();

        frameId++;
    }
   
    return 0;
}