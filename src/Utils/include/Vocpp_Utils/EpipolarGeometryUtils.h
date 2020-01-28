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
* /brief Calculate fundamental matrix such that transpose(in_leftPoints) * F * in_rightPoints = 0
* Here we fit the most generic model for a fundamental matrix assuming a non-zero rotation + translation.
* Algorithm in use is the 8-Point Algorithm
*
* \return true if successful, otherwise false
*/
bool CalculateFundamentalMatrix8pt(const std::vector<cv::Point2f>& in_leftPoints, const std::vector<cv::Point2f>& in_rightPoints, cv::Mat& out_fundMatrix);

} //namespace Utils
} //namespace VOCPP

