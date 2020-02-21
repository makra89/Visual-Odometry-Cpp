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
        std::vector<cv::Mat1f>& out_solutions)
{
    bool ret = true;

    if (in_pointCorrLeft.size() != in_pointCorrRight.size())
    {
        ret = false;
        std::cout << "[FullFundamentalMatrix]: Dimensions of vectors have to be equal " << std::endl;
    }
    else
    {
        cv::Mat1f transLeft;
        cv::Mat1f transRight;
        std::vector<cv::Point2f> normLeftPoints;
        std::vector<cv::Point2f> normRightPoints;
        normLeftPoints.reserve(in_pointCorrLeft.size());
        normRightPoints.reserve(in_pointCorrRight.size());

        Utils::NormalizePointSet(in_pointCorrLeft, normLeftPoints, transLeft);
        Utils::NormalizePointSet(in_pointCorrRight, normRightPoints, transRight);

        // Do this in double precision
        cv::Mat1d A = cv::Mat1d::zeros(static_cast<int>(normRightPoints.size()), 9);

        for (int it = 0; it < static_cast<int>(normRightPoints.size()); it++)
        {
            A(it, 0) = static_cast<double>(normRightPoints[it].x)* static_cast<double>(normLeftPoints[it].x);
            A(it, 1) = static_cast<double>(normRightPoints[it].x)* static_cast<double>(normLeftPoints[it].y);
            A(it, 2) = static_cast<double>(normRightPoints[it].x);
            A(it, 3) = static_cast<double>(normRightPoints[it].y)* static_cast<double>(normLeftPoints[it].x);
            A(it, 4) = static_cast<double>(normRightPoints[it].y)* static_cast<double>(normLeftPoints[it].y);
            A(it, 5) = static_cast<double>(normRightPoints[it].y);
            A(it, 6) = static_cast<double>(normLeftPoints[it].x);
            A(it, 7) = static_cast<double>(normLeftPoints[it].y);
            A(it, 8) = 1.0;
        }

        // Compute SVD
        cv::Mat1d U, D, V_t;
        cv::SVDecomp(A, D, U, V_t, cv::SVD::FULL_UV);

        // Reshape last row of V_t to fundamental matrix
        // Don't forget to transpose the matrix
        cv::Mat1d F;
        V_t.row(8).copyTo(F);
        F = F.reshape(0, 3).t();

        // Compute SVD of fundamental matrix and recalculate with smalles singular value = 0
        // This has to be done to ensure rank(F) = 2 --> det(F) = 0
        cv::SVDecomp(F, D, U, V_t, cv::SVD::FULL_UV);
        cv::Mat1d Diag = cv::Mat1d::zeros(3, 3);
        Diag(0, 0) = D(0, 0);
        Diag(1, 1) = D(1, 0);
        Diag(2, 2) = 0.0;
        F = U * (Diag * V_t);

        // Convert back to float
        cv::Mat1f fundMat;
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
    const cv::Mat1f& in_solution, const float in_errorTresh, std::vector<int>& out_inliers)
{
    out_inliers.clear();

    for (int i = 0; i < in_pointCorrLeft.size(); i++)
    {
        // We are using the symmetric epipolar distance here (SED)
        // See https://www.robots.ox.ac.uk/~vgg/publications/1998/Torr98c/torr98c.pdf
        // There it is called "Luongs distance"
        cv::Mat1f epiLineLeft = in_solution * Utils::Point2fToMatHomCoordinates(in_pointCorrRight[i]);
        cv::Mat1f epiLineRight = Utils::Point2fToMatHomCoordinates(in_pointCorrLeft[i]).t() * in_solution;

        const float scale = 1.0F / (std::pow(epiLineLeft(0, 0), 2) + std::pow(epiLineLeft(1, 0), 2)) +
            (1.0F / (std::pow(epiLineRight(0, 0), 2) + std::pow(epiLineRight(1, 0), 2)));

        cv::Mat1f dist = Utils::Point2fToMatHomCoordinates(in_pointCorrLeft[i]).t() * epiLineLeft;

        // Codimension 1 to R^4 --> divide by 3.84
        if ((std::pow(dist(0, 0), 2) * scale / 3.84F) < std::pow(in_errorTresh, 2))
        {
            out_inliers.push_back(i);
        }
    }
}

bool FullFundamentalMat8pt::DecomposeSolution(const cv::Mat1f& in_solution, const cv::Mat1f& in_calibMat, const std::vector<cv::Point2f>& in_pointCorrLeft,
    const std::vector<cv::Point2f>& in_pointCorrRight, cv::Mat1f& out_translation, cv::Mat1f& out_rotation)
{
    cv::Mat1f essentialMat = in_calibMat.t() * (in_solution * in_calibMat);

    cv::Mat1f invCalibMat = in_calibMat.inv();
    cv::Mat1f leftCamCoord = invCalibMat * Utils::Point2fToMatHomCoordinates(in_pointCorrLeft[0]);
    cv::Mat1f rightCamCoord = invCalibMat * Utils::Point2fToMatHomCoordinates(in_pointCorrRight[0]);
    
    bool ret = Utils::DecomposeEssentialMatrix(essentialMat, cv::Point2f(leftCamCoord(0,0) / leftCamCoord(2, 0), leftCamCoord(1, 0) / leftCamCoord(2, 0)),
        cv::Point2f(rightCamCoord(0, 0) / rightCamCoord(2, 0), rightCamCoord(1, 0) / rightCamCoord(2, 0)), out_translation, out_rotation);

    // Normalize translation
    out_translation = out_translation / cv::norm(out_translation);

    return ret;
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP