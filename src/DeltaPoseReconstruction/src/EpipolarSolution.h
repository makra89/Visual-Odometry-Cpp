/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_EPIPOLAR_SOLVER_H
#define VOCPP_EPIPOLAR_SOLVER_H

#include <Vocpp_Interface/Types.h>
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
bool RecoverPoseRansac(const std::vector<cv::Point2d>& in_correspondFirst, const std::vector<cv::Point2d>& in_correspondSecond,
        const cv::Mat1d& in_calibMat, std::vector<uint32_t>& out_inlierMatchIndices, cv::Mat1d& out_translation, 
        cv::Mat1d& out_rotation, std::vector<cv::Point3d>& out_triangulatedPoints);


} //namespace DeltaPoseReconstruction
} //namespace VOCPP

#endif /* VOCPP_EPIPOLAR_SOLVER_H */
