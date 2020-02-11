/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_PURE_TRANSLATION_MODEL_H
#define VOCPP_PURE_TRANSLATION_MODEL_H

#include <EpipolarModel.h>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

/**
  * /brief Tries to reconstruct a fundamental matrix consisting out of a pure translation
  */
class PureTranslationModel : public EpipolarModel
{
public:

    /**
      * /brief Constructor of model, needed correspondences: 2,  dimension = 3, 2 DOF
      */
    PureTranslationModel() : EpipolarModel(2, 3, 2)
    {
        m_type = EpipolarModel::Types::PureTranslationModel;
    }
    
    /**
      * /brief Compute model solution for given image correspondences. In dependency which
      * model is chosen the terms  "left" and "right" refer to either
      * x_left.T * F * x_right or x_left = H * x_right.
      * F and H are fundamental matrices and homographies, respectively.
      */
    virtual bool Compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        std::vector<cv::Mat>& out_solutions) override;

    /**
     * /brief Tests a particular model solution with a set of correspondendec and computes the inliers given an error treshold.
     * When comparing the error treshold value to the actual distance error measure,
     * the model has to scale this value by the 95% value of a chi^2 distribution
     * for either one or two dimensions (depends on the codimension of the variety)
     * The terms  "left" and "right" refer to either  x_left.T * F * x_right or x_left = H * x_right.
     * F and H are fundamental matrices and homographies, respectively.
     */
    virtual void Test(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        const cv::Mat& in_solution, const float in_errorTresh, std::vector<int>& out_inliers) override;

    /**
      * /brief Decompose a model solution into a translation and a rotation matrix
      * The rotation and translation is defined in such way that x_left = R * x_right + translation
      */
    virtual bool DecomposeSolution(const cv::Mat& in_solution, const cv::Mat& in_calibMat, const std::vector<cv::Point2f>& in_pointCorrLeft,
        const std::vector<cv::Point2f>& in_pointCorrRight, cv::Mat& out_translation, cv::Mat& out_rotation) override;
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP

#endif /* VOCPP_PURE_TRANSLATION_MODEL_H */
