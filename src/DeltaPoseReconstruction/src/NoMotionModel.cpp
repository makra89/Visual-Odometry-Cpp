/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <NoMotionModel.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <Vocpp_Utils/ConversionUtils.h>
#include <iostream>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

void NoMotionModel::Test(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
    const cv::Mat& in_solution, const float in_errorTresh, std::vector<int>& out_inliers)
{
    for (int i = 0; i < in_pointCorrLeft.size(); i++)
    {
        // Compute error as squared distance of left and right pixels
        const float error = std::pow(in_pointCorrLeft[i].x - in_pointCorrRight[i].x, 2) + std::pow(in_pointCorrLeft[i].y - in_pointCorrRight[i].y, 2);
      
        // Codimension 2 to R^4 --> divide by 5.99
        if (error < std::pow(in_errorTresh, 2))
        {
            out_inliers.push_back(i);
        }
    }

}

bool NoMotionModel::DecomposeSolution(const cv::Mat& in_solution, const cv::Mat& in_calibMat, const std::vector<cv::Point2f>& in_pointCorrLeft,
    const std::vector<cv::Point2f>& in_pointCorrRight, cv::Vec3f& out_translation, cv::Mat& out_rotation)
{
    // No motion --> no translation
    out_translation[0] = 0.0;
    out_translation[1] = 0.0;
    out_translation[2] = 0.0;

    // And no rotation
    out_rotation = cv::Mat::eye(3, 3, CV_32F);

    return true;
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP