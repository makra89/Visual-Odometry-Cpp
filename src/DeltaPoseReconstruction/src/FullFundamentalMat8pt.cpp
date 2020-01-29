/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <FullFundamentalMat8pt.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <iostream>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

bool FullFundamentalMat8pt::Compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        std::vector<cv::Mat>& out_solutions)
{
    bool ret = true;

    if (in_pointCorrLeft.size() != in_pointCorrRight.size())
    {
        ret = false;
        std::cout << "[CalculateFundamentalMatrix]: Dimensions of vectors have to be equal " << std::endl;
    }
    else
    {
        cv::Mat transLeft;
        cv::Mat transRight;
        std::vector<cv::Point2f> in_normLeftPoints;
        std::vector<cv::Point2f> in_normRightPoints;
        Utils::NormalizePointSet(in_pointCorrLeft, in_normLeftPoints, transLeft);
        Utils::NormalizePointSet(in_pointCorrRight, in_normRightPoints, transRight);

        cv::Mat A = cv::Mat::zeros(static_cast<int>(in_normRightPoints.size()), 9, CV_64F);

        for (int it = 0; it < static_cast<int>(in_normRightPoints.size()); it++)
        {
            A.at<double>(it, 0) = static_cast<double>(in_normRightPoints[it].x)* static_cast<double>(in_normLeftPoints[it].x);
            A.at<double>(it, 1) = static_cast<double>(in_normRightPoints[it].x)* static_cast<double>(in_normLeftPoints[it].y);
            A.at<double>(it, 2) = static_cast<double>(in_normRightPoints[it].x);
            A.at<double>(it, 3) = static_cast<double>(in_normRightPoints[it].y)* static_cast<double>(in_normLeftPoints[it].x);
            A.at<double>(it, 4) = static_cast<double>(in_normRightPoints[it].y)* static_cast<double>(in_normLeftPoints[it].y);
            A.at<double>(it, 5) = static_cast<double>(in_normRightPoints[it].y);
            A.at<double>(it, 6) = static_cast<double>(in_normLeftPoints[it].x);
            A.at<double>(it, 7) = static_cast<double>(in_normLeftPoints[it].y);
            A.at<double>(it, 8) = 1.0;
        }

        // Compute SVD
        cv::Mat U, D, V_t;
        cv::SVDecomp(A, D, U, V_t, cv::SVD::FULL_UV);

        // Reshape last row of V_t to fundamental matrix
        // Don't forget to transpose the matrix
        cv::Mat F;
        V_t.row(8).copyTo(F);
        F = F.reshape(0, 3).t();

        // Compute SVD of fundamental matrix and recalculate with smalles singular value = 0
        // This has to be done to ensure rank(F) = 2 --> det(F) = 0
        cv::SVDecomp(F, D, U, V_t, cv::SVD::FULL_UV);
        cv::Mat Diag = cv::Mat::zeros(3, 3, CV_64F);
        Diag.at<double>(0, 0) = D.at<double>(0, 0);
        Diag.at<double>(1, 1) = D.at<double>(1, 0);
        Diag.at<double>(2, 2) = 0.0;
        F = U * (Diag * V_t);

        // Convert back to float
        cv::Mat fundMat;
        F.convertTo(fundMat, CV_32F);

        // Apply the normalization transformations used for the image coordinates
        fundMat = transLeft.t() * fundMat * transRight;

        // And normalize the 2,2 element to 1.0
        if (std::abs(fundMat.at<float>(2, 2)) > 0.0)
        {
            fundMat = fundMat / fundMat.at<float>(2, 2);
        }

        out_solutions.push_back(fundMat);
    }

    return ret;
}

void FullFundamentalMat8pt::Test(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
    cv::Mat& in_solution, std::vector<int>& out_inliers)
{
    
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP