/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include <EpipolarModel.h>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

/**
* /brief Tries to reconstruct a full fundamental matrix from provided point matches. 8 correspondences are needed.
*/
class FullFundamentalMat8pt : public EpipolarModel
{
public:

    virtual bool compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        const std::vector<cv::Mat>& out_solutions);
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP
