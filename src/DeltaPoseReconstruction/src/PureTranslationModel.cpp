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
        std::vector<cv::Mat1f>& out_solutions)
{
    bool ret = true;

    if (in_pointCorrLeft.size() != in_pointCorrRight.size())
    {
        ret = false;
        std::cout << "[PureTranslationModel]: Dimensions of vectors have to be equal " << std::endl;
    }
    else
    {
        cv::Mat1f normTransform;
        
        // In theory we should normalize feature points for both images independently
        // However we want to guarantee a certain shape of the fundamental matrix.
        // This is only possible if we have the same transformation for left and right image
        std::vector< cv::Point2f> combinedPoints = in_pointCorrLeft;
        combinedPoints.insert(combinedPoints.end(), in_pointCorrRight.begin(), in_pointCorrRight.end());
        std::vector< cv::Point2f> normCombinedPoints;  
        Utils::NormalizePointSet(combinedPoints, normCombinedPoints, normTransform);
        std::vector<cv::Point2f> normLeftPoints(normCombinedPoints.begin(), normCombinedPoints.begin() + normCombinedPoints.size() / 2);
        std::vector<cv::Point2f> normRightPoints(normCombinedPoints.begin() + normCombinedPoints.size() / 2, normCombinedPoints.end());

        // Do this in double precision
        cv::Mat1d A = cv::Mat1d::zeros(static_cast<int>(normLeftPoints.size()), 3);

        for (int it = 0; it < static_cast<int>(normLeftPoints.size()); it++)
        {
            A(it, 0) = static_cast<double>(normLeftPoints[it].y) - static_cast<double>(normRightPoints[it].y);
            A(it, 1) = static_cast<double>(normRightPoints[it].x) - static_cast<double>(normLeftPoints[it].x);
            A(it, 2) = static_cast<double>(normLeftPoints[it].x) * static_cast<double>(normRightPoints[it].y)
            - static_cast<double>(normLeftPoints[it].y) * static_cast<double>(normRightPoints[it].x);
        }

        // Compute SVD
        cv::Mat1d U, D, V_t;
        cv::SVDecomp(A, D, U, V_t, cv::SVD::FULL_UV);

        cv::Mat1d F = cv::Mat1d::zeros(3, 3);
        F(1, 2) = V_t.row(2)(0, 0);
        F(2, 1) = -V_t.row(2)(0, 0);
        F(2, 0) = V_t.row(2)(0, 1);
        F(0, 2) = -V_t.row(2)(0, 1);
        F(0, 1) = V_t.row(2)(0, 2);
        F(1, 0) = -V_t.row(2)(0, 2);

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
        fundMat = normTransform.t() * fundMat * normTransform;

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
    const cv::Mat1f& in_solution, const float in_errorTresh, std::vector<int>& out_inliers)
{

    for (int i = 0; i < in_pointCorrLeft.size(); i++)
    {
        // We are using the symmetric epipolar distance here (SED)
        // See https://www.robots.ox.ac.uk/~vgg/publications/1998/Torr98c/torr98c.pdf
        // There it is called "Luongs distance"
        cv::Mat1f epiLineLeft = in_solution * Utils::Point2fToMatHomCoordinates(in_pointCorrRight[i]);
        // Note, this is a (1,3) vector!
        cv::Mat1f epiLineRight = Utils::Point2fToMatHomCoordinates(in_pointCorrLeft[i]).t() * in_solution;
        const float scale = 1.0F / (std::pow(epiLineLeft(0, 0), 2) + std::pow(epiLineLeft(1, 0), 2)) +
            (1.0F / (std::pow(epiLineRight(0, 0), 2) + std::pow(epiLineRight(0, 1), 2)));

        cv::Mat1f dist = Utils::Point2fToMatHomCoordinates(in_pointCorrLeft[i]).t() * epiLineLeft;

        // Codimension 1 to R^4 --> divide by 3.84
        if ((std::pow(dist(0, 0), 2) * scale / 3.84F) < std::pow(in_errorTresh, 2))
        {
            out_inliers.push_back(i);
        }
    }

}

bool PureTranslationModel::DecomposeSolution(const cv::Mat1f& in_solution, const cv::Mat1f& in_calibMat, const std::vector<cv::Point2f>& in_pointCorrLeft,
    const std::vector<cv::Point2f>& in_pointCorrRight, cv::Mat1f& out_translation, cv::Mat1f& out_rotation)
{
    cv::Mat1f essentialMat = in_calibMat.t() * (in_solution * in_calibMat);
    
    out_translation = cv::Mat1f::zeros(3, 1);

    // We want the translation from the left frame to the right frame
    out_translation(0, 0) = essentialMat(1, 2);
    out_translation(1, 0) = essentialMat(2, 0);
    out_translation(2, 0) = essentialMat(0, 1);

    out_translation = out_translation / cv::norm(out_translation);

    // No translation
    out_rotation = cv::Mat1f::eye(3, 3);

    return true;
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP