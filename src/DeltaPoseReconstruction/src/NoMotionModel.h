/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_NO_MOTION_MODEL_H
#define VOCPP_NO_MOTION_MODEL_H

#include <EpipolarModel.h>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

/**
  * /brief In this model we assume no motion at all
  */
class NoMotionModel : public EpipolarModel
{
public:

    /**
      * /brief Constructor of model, needed correspondences: 0,  dimension = 2, 0 DOF
      */
    NoMotionModel() : EpipolarModel(0, 2, 0)
    {
        m_type = EpipolarModel::Types::NoMotionModel;
    }
    
    /**
      * /brief Compute model solution for given image correspondences. In dependency which 
      * model is chosen the terms  "left" and "right" refer to either
      * x_left.T * F * x_right or x_left = H * x_right. 
      * F and H are fundamental matrices and homographies, respectively.
      */
    virtual bool Compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        std::vector<cv::Mat1f>& out_solutions) override
    {
        out_solutions.push_back(cv::Mat1f::eye(3, 3));
        return true;
    }

    /**
     * /brief Tests a particular model solution with a set of correspondendec and computes the inliers given an error treshold.
     * When comparing the error treshold value to the actual distance error measure,
     * the model has to scale this value by the 95% value of a chi^2 distribution
     * for either one or two dimensions (depends on the codimension of the variety)
     * The terms  "left" and "right" refer to either  x_left.T * F * x_right or x_left = H * x_right.
     * F and H are fundamental matrices and homographies, respectively.
     */
    virtual void Test(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        const cv::Mat1f& in_solution, const float in_errorTresh, std::vector<int>& out_inliers) override;

    /**
      * /brief Decompose a model solution into a translation and a rotation matrix
      * The rotation and translation is defined in such way that x_left = R * x_right + translation
      */
    virtual bool DecomposeSolution(const cv::Mat1f& in_solution, const cv::Mat1f& in_calibMat, const std::vector<cv::Point2f>& in_pointCorrLeft,
        const std::vector<cv::Point2f>& in_pointCorrRight, cv::Mat1f& out_translation, cv::Mat1f& out_rotation) override;
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP

#endif /* VOCPP_NO_MOTION_MODEL_H */
