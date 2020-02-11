/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <PureTranslationModel.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <Vocpp_Utils/ConversionUtils.h>
#include <iostream>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

bool PureTranslationModel::Compute(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
        std::vector<cv::Mat>& out_solutions)
{
    bool ret = true;

    if (in_pointCorrLeft.size() != in_pointCorrRight.size())
    {
        ret = false;
        std::cout << "[PureTranslationModel]: Dimensions of vectors have to be equal " << std::endl;
    }
    else
    {
        cv::Mat transLeft;
        cv::Mat transRight;

        cv::Mat A = cv::Mat::zeros(static_cast<int>(in_pointCorrLeft.size()), 3, CV_64F);

        for (int it = 0; it < static_cast<int>(in_pointCorrLeft.size()); it++)
        {
            A.at<double>(it, 0) = static_cast<double>(in_pointCorrLeft[it].y) - static_cast<double>(in_pointCorrRight[it].y);
            A.at<double>(it, 1) = static_cast<double>(in_pointCorrRight[it].x) - static_cast<double>(in_pointCorrLeft[it].x);
            A.at<double>(it, 2) = static_cast<double>(in_pointCorrLeft[it].x) * static_cast<double>(in_pointCorrRight[it].y)
            - static_cast<double>(in_pointCorrLeft[it].y) * static_cast<double>(in_pointCorrRight[it].x);
        }

        // Compute SVD
        cv::Mat U, D, V_t;
        cv::SVDecomp(A, D, U, V_t, cv::SVD::FULL_UV);

        cv::Mat F = cv::Mat::zeros(3, 3, CV_64F);
        F.at<double>(1, 2) = V_t.row(2).at<double>(0, 0);
        F.at<double>(2, 1) = -V_t.row(2).at<double>(0, 0);
        F.at<double>(2, 0) = V_t.row(2).at<double>(0, 1);
        F.at<double>(0, 2) = -V_t.row(2).at<double>(0, 1);
        F.at<double>(0, 1) = V_t.row(2).at<double>(0, 2);
        F.at<double>(1, 0) = -V_t.row(2).at<double>(0, 2);

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

        // And normalize matrix
        if (cv::norm(fundMat) > 0.0)
        {
            fundMat = fundMat / cv::norm(fundMat);
        }

        out_solutions.push_back(fundMat);

    }

    return ret;
}

void PureTranslationModel::Test(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
    const cv::Mat& in_solution, const float in_errorTresh, std::vector<int>& out_inliers)
{

    for (int i = 0; i < in_pointCorrLeft.size(); i++)
    {
        // We are using the symmetric epipolar distance here (SED)
        // See https://www.robots.ox.ac.uk/~vgg/publications/1998/Torr98c/torr98c.pdf
        // There it is called "Luongs distance"
        cv::Mat epiLineLeft = in_solution * Utils::Point2fToMatHomCoordinates(in_pointCorrRight[i]);
        cv::Mat epiLineRight = Utils::Point2fToMatHomCoordinates(in_pointCorrLeft[i]).t() * in_solution;

        const float scale = 1.0F / (std::pow(epiLineLeft.at<float>(0, 0), 2) + std::pow(epiLineLeft.at<float>(1, 0), 2)) +
            (1.0F / (std::pow(epiLineRight.at<float>(0, 0), 2) + std::pow(epiLineRight.at<float>(1, 0), 2)));

        cv::Mat dist = Utils::Point2fToMatHomCoordinates(in_pointCorrLeft[i]).t() * epiLineLeft;

        // Codimension 1 to R^4 --> divide by 3.84
        if ((std::pow(dist.at<float>(0, 0), 2) * scale / 3.84F) < std::pow(in_errorTresh, 2))
        {
            out_inliers.push_back(i);
        }
    }

}

bool PureTranslationModel::DecomposeSolution(const cv::Mat& in_solution, const cv::Mat& in_calibMat, const std::vector<cv::Point2f>& in_pointCorrLeft,
    const std::vector<cv::Point2f>& in_pointCorrRight, cv::Mat& out_translation, cv::Mat& out_rotation)
{
    cv::Mat essentialMat = in_calibMat.t() * (in_solution * in_calibMat);
    
    out_translation = cv::Mat::zeros(3, 1, CV_32F);

    // We want the translation from the left frame to the right frame
    out_translation.at<float>(0, 0) = essentialMat.at<float>(1, 2);
    out_translation.at<float>(1, 0) = essentialMat.at<float>(2, 0);
    out_translation.at<float>(2, 0) = essentialMat.at<float>(0, 1);

    // No translation
    out_rotation = cv::Mat::eye(3, 3, CV_32F);

    return true;
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP