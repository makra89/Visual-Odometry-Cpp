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

} //namespace Utils
} //namespace VOCPP

