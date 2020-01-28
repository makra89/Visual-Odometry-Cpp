/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/FrameRotations.h>
#include<opencv2/imgproc.hpp>

namespace VOCPP
{
namespace Utils
{

cv::Mat GetFrameRotationX(const float in_angleRad)
{
    cv::Mat frameRotX = cv::Mat::zeros(3, 3, CV_32F);
    
    frameRotX.at<float>(0, 0) = 1.0;
    
    frameRotX.at<float>(1, 1) = std::cos(in_angleRad);
    frameRotX.at<float>(1, 2) = std::sin(in_angleRad);
    frameRotX.at<float>(2, 1) = -std::sin(in_angleRad);
    frameRotX.at<float>(2, 2) = std::cos(in_angleRad);

    return frameRotX;
}

cv::Mat GetFrameRotationY(const float in_angleRad)
{
    cv::Mat frameRotY = cv::Mat::zeros(3, 3, CV_32F);

    frameRotY.at<float>(1, 1) = 1.0;
    
    frameRotY.at<float>(0, 0) = std::cos(in_angleRad);
    frameRotY.at<float>(0, 2) = -std::sin(in_angleRad);
    frameRotY.at<float>(2, 0) = std::sin(in_angleRad);
    frameRotY.at<float>(2, 2) = std::cos(in_angleRad);

    return frameRotY;
}

cv::Mat GetFrameRotationZ(const float in_angleRad)
{
    cv::Mat frameRotZ = cv::Mat::zeros(3, 3, CV_32F);

    frameRotZ.at<float>(2, 2) = 1.0;
    
    frameRotZ.at<float>(0, 0) = std::cos(in_angleRad);
    frameRotZ.at<float>(0, 1) = std::sin(in_angleRad);
    frameRotZ.at<float>(1, 0) = -std::sin(in_angleRad);
    frameRotZ.at<float>(1, 1) = std::cos(in_angleRad);

    return frameRotZ;
}

} //namespace Utils
} //namespace VOCPP
