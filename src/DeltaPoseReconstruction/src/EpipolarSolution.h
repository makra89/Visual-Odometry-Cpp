/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_EPIPOLAR_SOLVER_H
#define VOCPP_EPIPOLAR_SOLVER_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

/**
  * /brief Recover the rotation and translation using RANSAC
  * The reconstructed rotation and translation is defined in the following way: 
  * corrFirst = rot * (corrSecond - translation) (in homogenous image coordinates)
  * The translation propagates the second camera center to the first one and the rotation
  * aligns the two coordinate system
  */
bool RecoverPoseRansac(const std::vector<cv::Point2f>& in_correspondFirst, const std::vector<cv::Point2f>& in_correspondSecond,
        const cv::Mat1f& in_calibMat, std::vector<unsigned int>& out_inlierMatchIndices, cv::Mat1f& out_translation, 
        cv::Mat1f& out_rotation, std::vector<cv::Point3f>& out_triangulatedPoints);


} //namespace DeltaPoseReconstruction
} //namespace VOCPP

#endif /* VOCPP_EPIPOLAR_SOLVER_H */
