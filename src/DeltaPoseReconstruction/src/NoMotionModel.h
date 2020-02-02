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
      * /brief We have nothing to do here
      */
    virtual bool Compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        std::vector<cv::Mat>& out_solutions) override
    {
        out_solutions.push_back(cv::Mat::eye(3, 3, CV_32F));
        return true;
    }

    /**
      * /brief Test model solution given a set of image correspondences
      */
    virtual void Test(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        cv::Mat& in_solution, const float in_errorTresh, std::vector<int>& out_inliers) override;
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP

#endif /* VOCPP_NO_MOTION_MODEL_H */
