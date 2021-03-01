/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <EpipolarSolution.h>
#include <Vocpp_Utils/NumericalUtilities.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <opencv2/calib3d.hpp>

#include <iostream>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

bool RecoverPoseRansac(const std::vector<cv::Point2f>& in_correspondFirst, const std::vector<cv::Point2f>& in_correspondSecond,
        const cv::Mat1f& in_calibMat, std::vector<unsigned int>& out_inlierMatchIndices, cv::Mat1f &out_translation, cv::Mat1f& out_rotation)
{
    // Set from outside, or set here hardcoded?
    // This is the gaussian noise in the distance to the epipolar line
    // The models have to scale this value by the number of codimensions of their variety
    const float assumedDistanceError = 1.0;

    // We assume the essential mat (or fundamental mat) gives x_first.T * F * x_second
    // But OpenCv computes x_second.T * F * x_first --> reorder arguments
    cv::Mat inlierMask = cv::Mat::zeros(in_correspondFirst.size(), 2, CV_8UC1);
    cv::Mat1f essentialMat = cv::findEssentialMat(in_correspondSecond, in_correspondFirst, in_calibMat, cv::RANSAC, 0.998, assumedDistanceError, inlierMask);

    for (unsigned int i = 0U; i < in_correspondFirst.size(); i++)
    {
        if (inlierMask.at<uint8_t>(i) == 1) out_inlierMatchIndices.push_back(i);
    }

    bool ret = out_inlierMatchIndices.size() > 0U;
    
    if (ret && VOCPP::Utils::DecomposeEssentialMatrix(essentialMat, in_calibMat, in_correspondFirst[out_inlierMatchIndices[0]], in_correspondSecond[out_inlierMatchIndices[0]], out_translation, out_rotation))
    {
        // Normalize translation
        out_translation = out_translation / cv::norm(out_translation);


        // We want to output the translation vector from second to first camera center in the second camera coordinate system
        out_translation = -(out_rotation.t() * out_translation);
    }
        return ret;
}

} //namespace DeltaPoseReconstruction
} //namespace VOCPP