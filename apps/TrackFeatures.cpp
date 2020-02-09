/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include<Vocpp_Master/Master.h>
#include<Vocpp_Utils/Frame.h>
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
    cv::Mat calibMat = cv::Mat::eye(3, 3, CV_32F);
    calibMat.at<float>(0, 0) = 7.18856e+02;
    calibMat.at<float>(1, 1) = 7.18856e+02;
    calibMat.at<float>(0, 2) = 6.071928000000e+02;
    calibMat.at<float>(1, 2) = 1.852157000000e+02;
    VOCPP::Calibration::MonoCameraCalibration monoCalib(calibMat);

    // Load calibration
    voMaster.LoadCalibration(monoCalib);

    int frameId = 0U;
    for (auto imageName : imageNames)
    {
        // Convert the image to grayscale CV_32F, here from CV_8U to CV_32F
        cv::Mat grayScaleImg;
        cv::cvtColor(imread(imageName, 1), grayScaleImg, COLOR_BGR2GRAY);
        grayScaleImg.convertTo(grayScaleImg, CV_32F, 1.0 / 255.0);

        // Feed frame to Master
        Utils::Frame frame(std::move(grayScaleImg), frameId);
        voMaster.FeedNextFrame(frame);

        frameId++;
    }
   
    return 0;
}