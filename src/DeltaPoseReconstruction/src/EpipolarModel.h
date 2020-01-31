/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>


namespace VOCPP
{
namespace DeltaPoseReconstruction
{

/**
* /brief Reconstructs epipolar geometry out of provided feature point matches
*/
class EpipolarModel
{
public:
    
    /**
      * /brief Available epipolar geometry models
      */
    enum class Types
    {
        FullFundMat8pt,
            None
    };

    /**
      * /brief Constructor
      */
    EpipolarModel(int in_numCorrespondences) : m_numCorrespondences(in_numCorrespondences)
    {
    }

    /**
      * /brief Destructor
      */
    virtual ~EpipolarModel()
    {
    }
    
    /**
      * /brief Compute model solution for given image correspondences
      */
    virtual bool Compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight, 
            std::vector<cv::Mat>& out_solutions) = 0;

    /**
      * /brief Tests a particular model solution with a set of correspondendec and computes the inliers given an error treshold.
      * When comparing the error treshold value to the actual distance error measure, 
      * the model has to scale this value by the 95% value of a chi^2 distribution
      * for either one or two dimensions (depends on the codimension of the variety)
      */
    virtual void Test(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        cv::Mat& in_solution, const float in_errorTresh, std::vector<int>& out_inliers) = 0;

    /**
      * /brief Get number of necessary image correspondences to compute a solution for the model
      */
    int GetNumCorrespondences()
    {
        return m_numCorrespondences;
    }

protected:

    const int m_numCorrespondences; ///< number of correspondences needed by this model
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP


