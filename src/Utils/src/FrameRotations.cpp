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

cv::Mat1f GetFrameRotationX(const float in_angleRad)
{
    cv::Mat1f frameRotX = cv::Mat1f::zeros(3, 3);
    
    frameRotX.at<float>(0, 0) = 1.0;
    
    frameRotX(1, 1) = std::cos(in_angleRad);
    frameRotX(1, 2) = std::sin(in_angleRad);
    frameRotX(2, 1) = -std::sin(in_angleRad);
    frameRotX(2, 2) = std::cos(in_angleRad);

    return frameRotX;
}

cv::Mat1f GetFrameRotationY(const float in_angleRad)
{
    cv::Mat1f frameRotY = cv::Mat1f::zeros(3, 3);

    frameRotY.at<float>(1, 1) = 1.0;
    
    frameRotY(0, 0) = std::cos(in_angleRad);
    frameRotY(0, 2) = -std::sin(in_angleRad);
    frameRotY(2, 0) = std::sin(in_angleRad);
    frameRotY(2, 2) = std::cos(in_angleRad);

    return frameRotY;
}

cv::Mat1f GetFrameRotationZ(const float in_angleRad)
{
    cv::Mat1f frameRotZ = cv::Mat1f::zeros(3, 3);

    frameRotZ(2, 2) = 1.0;
    
    frameRotZ(0, 0) = std::cos(in_angleRad);
    frameRotZ(0, 1) = std::sin(in_angleRad);
    frameRotZ(1, 0) = -std::sin(in_angleRad);
    frameRotZ(1, 1) = std::cos(in_angleRad);

    return frameRotZ;
}

cv::Vec3f ExtractRollPitchYaw(const cv::Mat1f& in_rotationMat)
{
    cv::Vec3f eulerAngles;
    // Roll angle
    eulerAngles[0] = std::atan2(in_rotationMat(1, 2), in_rotationMat(2, 2));
    const float c2 = std::sqrt(std::pow(in_rotationMat(0, 0), 2) + std::pow(in_rotationMat(0, 1), 2));
    // Pitch angle
    eulerAngles[1] = std::atan2(-in_rotationMat(0, 2), c2);
    const float s1 = std::sin(eulerAngles[0]);
    const float c1 = std::cos(eulerAngles[0]);
    // Yaw angle
    eulerAngles[2] = std::atan2(s1 * in_rotationMat(2, 0) - c1 * in_rotationMat(1, 0), 
        c1 * in_rotationMat(1, 1) - s1 * in_rotationMat(2, 1));

    return eulerAngles;
}

} //namespace Utils
} //namespace VOCPP
