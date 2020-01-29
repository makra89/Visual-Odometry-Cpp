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

    EpipolarModel(int in_numCorrespondences)
    {
        m_numCorrespondences = in_numCorrespondences;
    }

    virtual ~EpipolarModel()
    {
    }
    
    virtual bool Compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight, 
            std::vector<cv::Mat>& out_solutions) = 0;

    virtual void Test(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        cv::Mat& in_solution, std::vector<int>& out_inliers) = 0;

    int GetNumCorrespondences()
    {
        return m_numCorrespondences;
    }

protected:

    int m_numCorrespondences; ///< number of correspondences needed by this model
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP


