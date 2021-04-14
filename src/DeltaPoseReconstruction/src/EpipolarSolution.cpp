/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <EpipolarSolution.h>
#include <Vocpp_Utils/NumericalUtilities.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <opencv2/calib3d/calib3d.hpp>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

bool RecoverPoseRansac(const std::vector<cv::Point2d>& in_correspondFirst, const std::vector<cv::Point2d>& in_correspondSecond,
        const cv::Mat1d& in_calibMat, std::vector<uint32_t>& out_inlierMatchIndices, cv::Mat1d &out_translation, cv::Mat1d& out_rotation, std::vector<cv::Point3d>& out_triangulatedPoints)
{
    // Set from outside, or set here hardcoded?
    // This is the gaussian noise in the distance to the epipolar line
    // The models have to scale this value by the number of codimensions of their variety
    const double assumedDistanceError = 0.5;

    // We assume the essential mat (or fundamental mat) gives x_first.T * F * x_second
    // But OpenCv computes x_second.T * F * x_first --> reorder arguments
    cv::Mat inlierMask = cv::Mat::zeros(static_cast<int32_t>(in_correspondFirst.size()), 2, CV_8UC1);
    cv::Mat essentialMat = cv::findEssentialMat(in_correspondSecond, in_correspondFirst, in_calibMat, cv::RANSAC, 0.999, assumedDistanceError, inlierMask);
    // It might happen that OpenCV cannot determine an essential mat --> check that the returned matrix has the expected dimensions
    const bool retOk = (essentialMat.cols == 3) && (essentialMat.rows == 3);

    if (retOk)
    {
        for (uint32_t i = 0U; i < in_correspondFirst.size(); i++)
        {
            if (inlierMask.at<uint8_t>(i) == 1) out_inlierMatchIndices.push_back(i);
        }

        std::vector<cv::Point3d> triangulatedPoints;
        if (out_inlierMatchIndices.size() > 0U && VOCPP::Utils::DecomposeEssentialMatrix(essentialMat, in_calibMat, in_correspondFirst, in_correspondSecond,
            out_inlierMatchIndices, out_translation, out_rotation, out_triangulatedPoints))
        {
            // For refinement we need at least 4 points
            if (out_triangulatedPoints.size() >= 4U)
            {
                cv::Mat rod;
                cv::Rodrigues(out_rotation, rod);

                std::vector<cv::Point2d> inlierFirst;
                for (uint32_t idx = 0U; idx < out_inlierMatchIndices.size(); idx++)
                {
                    inlierFirst.push_back(in_correspondFirst[out_inlierMatchIndices[idx]]);
                }

                // Try to refine the current pose by PnP
                cv::solvePnP(out_triangulatedPoints, inlierFirst, in_calibMat, cv::Mat(), rod, out_translation, true, cv::SOLVEPNP_ITERATIVE);

                cv::Rodrigues(rod, out_rotation);
            }

            // Normalize translation
            out_translation = out_translation / cv::norm(out_translation);

            // We want to output the translation vector from second to first camera center in the second camera coordinate system
            out_translation = -(out_rotation.t() * out_translation);
        }
    }

    return retOk && out_inlierMatchIndices.size() > 0U;
}

} //namespace DeltaPoseReconstruction
} //namespace VOCPP
