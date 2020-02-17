/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{
namespace Utils
{

/**
* /brief Get coordinate transformation matrix consisting out of a rotation around X axis.
Note: This is a rotation of a coordinate frame, not of a vector in a fixed coordinate frame!
*
* \param[in] in_angleRad rotation angle in radians
* \returns rotation matrix
*/
cv::Mat GetFrameRotationX(const float in_angleRad);

/**
* /brief Get coordinate transformation matrix consisting out of a rotation around Y axis.
Note: This is a rotation of a coordinate frame, not of a vector in a fixed coordinate frame!
*
* \param[in] in_angleRad rotation angle in radians
* \returns rotation matrix
*/
cv::Mat GetFrameRotationY(const float in_angleRad);

/**
* /brief Get coordinate transformation matrix consisting out of a rotation around Z axis.
Note: This is a rotation of a coordinate frame, not of a vector in a fixed coordinate frame!
*
* \param[in] in_angleRad rotation angle in radians
* \returns rotation matrix
*/
cv::Mat GetFrameRotationZ(const float in_angleRad);

/**
* /brief For a given 3D rotation matrix calculate yaw, roll and pitch angle in such way that
* in_rotationMat = R_x(roll) * R_y(pitch) * R_z(yaw)
* We start from the right-handed world coordinate system with z pointing upwards and rotate
* around the axis Z, Y', X''. The prime indicate that we rotate around the resulting coordinate axes.
* It is assumed that the rotation matrix rotates the coordinate system NOT a vector!
* Algorithm has been taken from https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
*
* \param[in] in_angleRad rotation angle in radians
* \returns rotation matrix
*/
cv::Vec3f ExtractRollPitchYaw(const cv::Mat& in_rotationMat);

} //namespace Utils
} //namespace VOCPP

