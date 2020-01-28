/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>


namespace VOCPP
{
namespace DeltaPoseReconstruction
{

/**
* /brief Available epipolar geometry models
*/
enum EpipolarModelTypes
{
    FullFundMat8pt
};

/**
* /brief Reconstructs epipolar geometry out of provided feature point matches
*/
class EpipolarModel
{
public:
    
virtual ~EpipolarModel()
{
}
    
virtual bool compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight, 
        std::vector<cv::Mat>& out_solutions) = 0;
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP


