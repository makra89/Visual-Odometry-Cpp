/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_EPIPOLAR_MODEL_H
#define VOCPP_EPIPOLAR_MODEL_H

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
        PureTranslationModel,
        NoMotionModel,
        None
    };

    /**
      * /brief Constructor
      */
    EpipolarModel(int in_numCorrespondences, int in_dimensionModel, int in_degreesOfFreedom) : 
        m_numCorrespondences(in_numCorrespondences), 
        m_degreesOfFreedom(in_degreesOfFreedom),
        m_dimensionModel(in_dimensionModel)
    {
    }

    /**
      * /brief Destructor
      */
    virtual ~EpipolarModel()
    {
    }
    
    /**
      * /brief Compute model solution for given image correspondences. In dependency which 
      * model is chosen the terms  "left" and "right" refer to either
      * x_left.T * F * x_right or x_left = H * x_right. 
      * F and H are fundamental matrices and homographies, respectively.
      */
    virtual bool Compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight, 
            std::vector<cv::Mat>& out_solutions) = 0;

    /**
      * /brief Tests a particular model solution with a set of correspondendec and computes the inliers given an error treshold.
      * When comparing the error treshold value to the actual distance error measure, 
      * the model has to scale this value by the 95% value of a chi^2 distribution
      * for either one or two dimensions (depends on the codimension of the variety)
      * The terms  "left" and "right" refer to either  x_left.T * F * x_right or x_left = H * x_right.
      * F and H are fundamental matrices and homographies, respectively.
      */
    virtual void Test(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        const cv::Mat& in_solution, const float in_errorTresh, std::vector<int>& out_inliers) = 0;

    /**
      * /brief Decompose a model solution into a translation and a rotation matrix
      * The rotation and translation is defined in such way that x_left = R * x_right + translation
      */
    virtual bool DecomposeSolution(const cv::Mat& in_solution, const cv::Mat& in_calibMat, const std::vector<cv::Point2f>& in_pointCorrLeft,
        const std::vector<cv::Point2f>& in_pointCorrRight, cv::Mat& out_translation, cv::Mat& out_rotation) = 0;

    /**
      * /brief Get number of necessary image correspondences to compute a solution for the model
      */
    int GetNumCorrespondences()
    {
        return m_numCorrespondences;
    }

    EpipolarModel::Types GetModelType()
    {
        return m_type;
    }

    /**
      * /brief Compute Plunder score based on number of inliers, outliers and degrees of freedom
      * This score can be used to compare which epipolar model to choose.
      */
    int ComputePlunderScore(const int in_numInliers, const int in_numOutliers)
    {
        return in_numInliers * m_dimensionModel + 4 * in_numOutliers + m_degreesOfFreedom;
    }

protected:

    const int m_numCorrespondences; ///< number of correspondences needed by this model
    const int m_degreesOfFreedom; ///< degrees of freedom of the model
    const int m_dimensionModel; ///< dimension of the variety defined by the model
    EpipolarModel::Types m_type;
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP


#endif /* VOCPP_EPIPOLAR_MODEL_H */
