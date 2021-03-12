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
* /brief Convert a cv::Point2d (a,b) to a cv::Mat1d containing the homogenous coordinates of the point (a,b,1)
*/
cv::Mat1d Point2dToMatHomCoordinates(const cv::Point2d& in_point);

/**
* /brief Convert a cv::Point3d (a,b, c) to a cv::Mat1d containing the homogenous coordinates of the point (a,b,c, 1)
*/
cv::Mat1d Point3dToMatHomCoordinates(const cv::Point3d& in_point);

} //namespace Utils
} //namespace VOCPP

