/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <FullFundamentalMat8pt.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <Vocpp_Utils/ConversionUtils.h>
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
        std::cout << "[FullFundamentalMatrix]: Dimensions of vectors have to be equal " << std::endl;
    }
    else
    {
        cv::Mat transLeft;
        cv::Mat transRight;
        std::vector<cv::Point2f> normLeftPoints;
        std::vector<cv::Point2f> normRightPoints;
        normLeftPoints.reserve(in_pointCorrLeft.size());
        normRightPoints.reserve(in_pointCorrRight.size());

        Utils::NormalizePointSet(in_pointCorrLeft, normLeftPoints, transLeft);
        Utils::NormalizePointSet(in_pointCorrRight, normRightPoints, transRight);

        cv::Mat A = cv::Mat::zeros(static_cast<int>(normRightPoints.size()), 9, CV_64F);

        for (int it = 0; it < static_cast<int>(normRightPoints.size()); it++)
        {
            A.at<double>(it, 0) = static_cast<double>(normRightPoints[it].x)* static_cast<double>(normLeftPoints[it].x);
            A.at<double>(it, 1) = static_cast<double>(normRightPoints[it].x)* static_cast<double>(normLeftPoints[it].y);
            A.at<double>(it, 2) = static_cast<double>(normRightPoints[it].x);
            A.at<double>(it, 3) = static_cast<double>(normRightPoints[it].y)* static_cast<double>(normLeftPoints[it].x);
            A.at<double>(it, 4) = static_cast<double>(normRightPoints[it].y)* static_cast<double>(normLeftPoints[it].y);
            A.at<double>(it, 5) = static_cast<double>(normRightPoints[it].y);
            A.at<double>(it, 6) = static_cast<double>(normLeftPoints[it].x);
            A.at<double>(it, 7) = static_cast<double>(normLeftPoints[it].y);
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

        // And normalize matrix
        if (cv::norm(fundMat) > 0.0)
        {
            fundMat = fundMat / cv::norm(fundMat);
        }

        out_solutions.push_back(fundMat);
    }

    return ret;
}

void FullFundamentalMat8pt::Test(const std::vector<cv::Point2f>& in_pointCorrLeft, const std::vector<cv::Point2f>& in_pointCorrRight,
    const cv::Mat& in_solution, const float in_errorTresh, std::vector<int>& out_inliers)
{
    out_inliers.clear();

    for (int i = 0; i < in_pointCorrLeft.size(); i++)
    {
        // We are using the symmetric epipolar distance here (SED)
        // See https://www.robots.ox.ac.uk/~vgg/publications/1998/Torr98c/torr98c.pdf
        // There it is called "Luongs distance"
        cv::Mat epiLineLeft = in_solution * Utils::Point2fToMatHomCoordinates(in_pointCorrRight[i]);
        cv::Mat epiLineRight = Utils::Point2fToMatHomCoordinates(in_pointCorrLeft[i]).t() * in_solution;

        const float scale = static_cast<float>(1.0) / (std::pow(epiLineLeft.at<float>(0, 0), 2) + std::pow(epiLineLeft.at<float>(1, 0), 2)) +
            (static_cast<float>(1.0) / (std::pow(epiLineRight.at<float>(0, 0), 2) + std::pow(epiLineRight.at<float>(1, 0), 2)));

        cv::Mat dist = Utils::Point2fToMatHomCoordinates(in_pointCorrLeft[i]).t() * epiLineLeft;

        // Codimension 1 to R^4 --> divide by 3.84
        if ((std::pow(dist.at<float>(0, 0), 2) * scale / static_cast<float>(3.84)) < std::pow(in_errorTresh, 2))
        {
            out_inliers.push_back(i);
        }
    }
}

bool FullFundamentalMat8pt::DecomposeSolution(const cv::Mat& in_solution, const cv::Mat& in_calibMat, const std::vector<cv::Point2f>& in_pointCorrLeft,
    const std::vector<cv::Point2f>& in_pointCorrRight, cv::Vec3f& out_translation, cv::Mat& out_rotation)
{
    cv::Mat essentialMat = in_calibMat.t() * (in_solution * in_calibMat);

    cv::Mat invCalibMat = in_calibMat.inv();
    cv::Mat leftCamCoord = invCalibMat * Utils::Point2fToMatHomCoordinates(in_pointCorrLeft[0]);
    cv::Mat rightCamCoord = invCalibMat * Utils::Point2fToMatHomCoordinates(in_pointCorrRight[0]);

    return Utils::DecomposeEssentialMatrix(essentialMat, cv::Point2f(leftCamCoord.at<float>(0,0) / leftCamCoord.at<float>(2, 0), leftCamCoord.at<float>(1, 0) / leftCamCoord.at<float>(2, 0)),
        cv::Point2f(rightCamCoord.at<float>(0, 0) / rightCamCoord.at<float>(2, 0), rightCamCoord.at<float>(1, 0) / rightCamCoord.at<float>(2, 0)), out_translation, out_rotation);
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP