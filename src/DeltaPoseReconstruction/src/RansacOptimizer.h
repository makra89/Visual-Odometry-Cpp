/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include <EpipolarModel.h>

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>


namespace VOCPP
{
namespace DeltaPoseReconstruction
{

/**
  * /brief Tries to minimize the error of each EpipolarModel given a set of corresponding image points
  * and additionally selects the model which fits best to the observed correspondences
  */
class RansacOptimizer
{
public:
    
    int Run(const std::vector<EpipolarModel*>& in_testedModels, const std::vector<cv::Point2f>& in_correspondFirst, const std::vector<cv::Point2f>& in_correspondSecond);

};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP


