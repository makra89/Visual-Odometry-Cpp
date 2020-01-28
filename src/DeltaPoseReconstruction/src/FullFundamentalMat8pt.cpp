/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <FullFundamentalMat8pt.h>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

bool FullFundamentalMat8pt::compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        const std::vector<cv::Mat>& out_solutions)
{
    bool ret = true;


    return ret;
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP