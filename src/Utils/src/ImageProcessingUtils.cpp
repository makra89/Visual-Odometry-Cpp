/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <Vocpp_Utils/ConversionUtils.h>
#include<opencv2/imgproc.hpp>
#include<iostream>

namespace VOCPP
{
namespace Utils
{

cv::Mat GetWindowKernel(const int size)
{
    return cv::Mat::ones(size, size, CV_32F);
}

void ApplyKernelToImage(const cv::Mat& in_image, const cv::Mat& in_kernel, cv::Mat& out_image)
{
    cv::filter2D(in_image, out_image, CV_32F, in_kernel);
}

void Compute2DGradients(const cv::Mat& in_image, cv::Mat& out_gradX, cv::Mat& out_gradY)
{ 
    // Compute gradient using sobel operators
    // Sobel operator can be replaced by two convolutions
    float dataRowX[3] = { -1.0, 0.0, 1.0 };
    cv::Mat sobelRowX = cv::Mat(1, 3, CV_32F, dataRowX);
    float dataColX[3] = { 1.0, 2.0, 1.0 };
    cv::Mat sobelColX = cv::Mat(3, 1, CV_32F, dataColX);
    cv::sepFilter2D(in_image, out_gradX, CV_32F, sobelRowX, sobelColX);

    float dataRowY[3] = { 1.0, 2.0, 1.0 };
    cv::Mat sobelRowY = cv::Mat(1, 3, CV_32F, dataRowY);
    float dataColY[3] = { -1.0, 0.0, 1.0 };
    cv::Mat sobelColY = cv::Mat(3, 1, CV_32F, dataColY);
    cv::sepFilter2D(in_image, out_gradY, CV_32F, sobelRowY, sobelColY);
}

void ExtractLocalMaxima(const cv::Mat& in_image, const int in_distance, std::vector<cv::Point2f>& out_localMaxima, const int in_subPixelCalculationDistance)
{
    cv::Mat dilatedImage;
    cv::Mat kernel = GetWindowKernel(2 * in_distance + 1);
    cv::dilate(in_image, dilatedImage, kernel);

    const int patchSize = 1;
    for (int i = 0; i < in_image.size[1]; i++)
    {
        for (int j = 0; j < in_image.size[0]; j++)
        {
            bool greaterZero = in_image.at<float>(j, i) > 0.0;
            // If image value is equal to the dilated value (maximum value of surrounding pixels)
            // this pixel has the maximum value
            if (greaterZero && in_image.at<float>(j, i) == dilatedImage.at<float>(j, i))
            {

                out_localMaxima.push_back(cv::Point2f(static_cast<float>(i), static_cast<float>(j)));
            }
        }
    }

    // If specified, try to calculate sub-pixel position by looking at centroids
    if (in_subPixelCalculationDistance > 0U)
    {
        for (auto pos = out_localMaxima.begin(); pos != out_localMaxima.end(); pos++)
        {
            const int patchDim = 2 * in_subPixelCalculationDistance + 1U;
            cv::Mat patch = cv::Mat::zeros(patchDim, patchDim, CV_32F);
            bool subPixSuccess = ExtractImagePatchAroundPixelPos(in_image, patch, in_subPixelCalculationDistance, static_cast<int>(pos->x), static_cast<int>(pos->y));

            if (subPixSuccess)
            {
                cv::Moments patchMoments = cv::moments(patch);

                pos->x = pos->x - in_subPixelCalculationDistance + static_cast<float>(patchMoments.m10 / patchMoments.m00);
                pos->y = pos->y - in_subPixelCalculationDistance + static_cast<float>(patchMoments.m01 / patchMoments.m00);
            }
        }
    }

}

bool ExtractImagePatchAroundPixelPos(const cv::Mat& in_image, cv::Mat& out_patch, const int in_distanceAroundCenter, const int in_pixelPosX, const int in_pixelPosY)
{
    bool ret = true;

    // First check whether all coordinates are within range of in_image
    const bool inRangeX = static_cast<int>(in_pixelPosX - in_distanceAroundCenter) >= 0 && static_cast<int>(in_pixelPosX + in_distanceAroundCenter) < in_image.size[1];
    const bool inRangeY = static_cast<int>(in_pixelPosY - in_distanceAroundCenter) >= 0 && static_cast<int>(in_pixelPosY + in_distanceAroundCenter) < in_image.size[0];
    // Then check if dimensions of provided patch matrix are correct
    const bool correctPatchDimX = out_patch.size[1] == (2 * in_distanceAroundCenter + 1);
    const bool correctPatchDimY = out_patch.size[0] == (2 * in_distanceAroundCenter + 1);

    if (inRangeX && inRangeY && correctPatchDimX && correctPatchDimY)
    {
        for (int i = 0; i < out_patch.size[1]; i++)
        {
            for (int j = 0; j < out_patch.size[0]; j++)
            {
                int imagePosX = in_pixelPosX - in_distanceAroundCenter + i;
                int imagePosÝ = in_pixelPosY - in_distanceAroundCenter + j;
                out_patch.at<float>(j, i) = in_image.at<float>(in_pixelPosY - in_distanceAroundCenter + j, in_pixelPosX - in_distanceAroundCenter + i);
            }
        }
    }
    else if (!correctPatchDimX || !correctPatchDimY)
    {
        ret = false;
        std::cout << "[ExtractImagePatchAroundPixelPos]: The dimension of the patch matrix are wrong" << std::endl;
    }
    else
    {
        ret = false;
    }

    return ret;

}

void ComputeMatchingPoints(const std::vector<cv::KeyPoint>& in_left, const std::vector<cv::KeyPoint>& in_right, const std::vector<cv::DMatch>& in_matchesLeftRight,
    const int in_imgIdx, std::vector<cv::Point2f>& out_leftPoints, std::vector<cv::Point2f>& out_rightPoints)
{
    for (auto match : in_matchesLeftRight)
    {
        if (match.imgIdx == in_imgIdx)
        {
            out_leftPoints.push_back(in_left[match.queryIdx].pt);
            out_rightPoints.push_back(in_right[match.trainIdx].pt);
        }
    }
}

void NormalizePointSet(const std::vector<cv::Point2f>& in_points, std::vector<cv::Point2f>& out_normPoints, cv::Mat& out_transform)
{
    float meanX = 0, meanY = 0;
    for (auto element : in_points)
    {
        meanX += static_cast<float>(element.x / in_points.size());
        meanY += static_cast<float>(element.y / in_points.size());
    }

    float meanDist = 0;
    for (auto element : in_points)
    {
        const float shiftedX = static_cast<float>(element.x) - meanX;
        const float shiftedY = static_cast<float>(element.y) - meanY;

        meanDist += ((std::pow(shiftedX, 2) + std::pow(shiftedY, 2)) / in_points.size());
    }

    const float scale = std::sqrt(2.0F) / std::sqrt(meanDist);

    for (auto element : in_points)
    {
        const float normX = scale * (static_cast<float>(element.x) - meanX);
        const float normY = scale * (static_cast<float>(element.y) - meanY);

        out_normPoints.push_back(cv::Point2f(normX, normY));
    }

    out_transform = cv::Mat::zeros(3, 3, CV_32F);
    out_transform.at<float>(0, 0) = scale;
    out_transform.at<float>(1, 1) = scale;
    out_transform.at<float>(0, 2) = -scale * meanX;
    out_transform.at<float>(1, 2) = -scale * meanY;
    out_transform.at<float>(2, 2) = 1.0;
}

bool GetProjectionMatrix(const cv::Mat& in_rotationMatrix, const cv::Mat& in_translation, cv::Mat& out_projMat)
{
    bool ret = true;

    if (in_rotationMatrix.rows != 3 || in_rotationMatrix.cols != 3)
    {
        std::cout << "[CalculateProjectionMatrix]: Rotation matrix has to be 3x3" << std::endl;
        ret = false;
    }
    else if (in_translation.rows != 3 || in_translation.cols != 1)
    {
        std::cout << "[CalculateProjectionMatrix]: translation matrix has to be 3x1" << std::endl;
        ret = false;
    }
    else
    {
        cv::hconcat(in_rotationMatrix, in_translation, out_projMat);
    }

    return ret;
}

void GetCrossProductMatrix(const cv::Vec3f& in_vec, cv::Mat& out_crossMat)
{
    out_crossMat = cv::Mat::zeros(3, 3, CV_32F);
    out_crossMat.at<float>(0, 1) = -in_vec[2];
    out_crossMat.at<float>(0, 2) = in_vec[1];
    out_crossMat.at<float>(1, 0) = in_vec[2];
    out_crossMat.at<float>(1, 2) = -in_vec[0];
    out_crossMat.at<float>(2, 0) = -in_vec[1];
    out_crossMat.at<float>(2, 1) = in_vec[0];
}

bool DecomposeEssentialMatrix(const cv::Mat& in_essentialMat, const cv::Point2f& in_matchPointLeft,
    const cv::Point2f& in_matchPointRight, cv::Mat& out_translation, cv::Mat& out_rotMatrix)
{
    // Compute SVD
    cv::Mat U, D, V_t;
    cv::SVDecomp(in_essentialMat, D, U, V_t, cv::SVD::FULL_UV);
   
    // Set smallest singular value to zero and recompute SVD
    cv::Mat newDiag = cv::Mat::zeros(3, 3, CV_32F);
    newDiag.at<float>(0, 0) = D.at<float>(0, 0);
    newDiag.at<float>(1, 1) = D.at<float>(1, 0);
    cv::Mat newEssentialMat = U * (newDiag * V_t);
    cv::SVDecomp(newEssentialMat, D, U, V_t, cv::SVD::FULL_UV);

    cv::Mat Y = cv::Mat::zeros(3, 3, CV_32F);
    Y.at<float>(0, 1) = -1.0;
    Y.at<float>(1, 0) = 1.0;
    Y.at<float>(2, 2) = 1.0;

    // Two solutions for rotation matrix
    cv::Mat R1 = U * (Y * V_t);
    cv::Mat R2 = U * (Y.t() * V_t);
    if(cv::determinant(R1) < 0) R1 = -R1;
    if(cv::determinant(R2) < 0) R2 = -R2;

    // And two for translation
    cv::Mat t1 = U.col(2);
    cv::Mat t2 = -t1;

    // Right projection matrix is trivial one
    cv::Mat projMatRight;
    GetProjectionMatrix(cv::Mat::eye(3, 3, CV_32F), cv::Mat::zeros(3, 1, CV_32F), projMatRight);

    // Four different projection matrices for right one
    cv::Mat projMatLeft1;
    GetProjectionMatrix(R1, t1, projMatLeft1);
    cv::Mat projMatLeft2;
    GetProjectionMatrix(R2, t1, projMatLeft2);
    cv::Mat projMatLeft3;
    GetProjectionMatrix(R1, t2, projMatLeft3);
    cv::Mat projMatLeft4;
    GetProjectionMatrix(R2, t2, projMatLeft4);

    std::vector<cv::Mat> projectionMatCandidates;
    projectionMatCandidates.push_back(projMatLeft1);
    projectionMatCandidates.push_back(projMatLeft2);
    projectionMatCandidates.push_back(projMatLeft3);
    projectionMatCandidates.push_back(projMatLeft4);
    
    // Check which of the solutions is the correct one by demanding that the triangulated point is in front of both cameras
    for (auto candidate : projectionMatCandidates)
    {
        cv::Point3f triangPoint;
        PointTriangulationLinear(candidate, projMatRight, in_matchPointLeft, in_matchPointRight, triangPoint);
        cv::Mat projPointLeft = candidate * VOCPP::Utils::Point3fToMatHomCoordinates(triangPoint);
        if (triangPoint.z >= 0.0 && projPointLeft.at<float>(2,0) >= 0)
        {
            candidate(cv::Range(0, 3), cv::Range(0, 3)).copyTo(out_rotMatrix);
            candidate(cv::Range(0, 3), cv::Range(3, 4)).copyTo(out_translation);
        }
    }
  

    return false;
}

bool PointTriangulationLinear(const cv::Mat& in_projMatLeft, const cv::Mat& in_projMatRight, const cv::Point2f& in_cameraCoordLeft,
    const cv::Point2f& in_cameraCoordRight, cv::Point3f& out_triangulatedPoint)
{
    cv::Mat A = cv::Mat::zeros(4, 4, CV_32F);

    // Construct matrix to decompose (see for example https://www.uio.no/studier/emner/matnat/its/nedlagte-emner/UNIK4690/v16/forelesninger/lecture_7_2-triangulation.pdf)
    A.at<float>(0, 0) = in_cameraCoordLeft.y * in_projMatLeft.at<float>(2, 0) - in_projMatLeft.at<float>(1, 0);
    A.at<float>(0, 1) = in_cameraCoordLeft.y * in_projMatLeft.at<float>(2, 1) - in_projMatLeft.at<float>(1, 1);
    A.at<float>(0, 2) = in_cameraCoordLeft.y * in_projMatLeft.at<float>(2, 2) - in_projMatLeft.at<float>(1, 2);
    A.at<float>(0, 3) = in_cameraCoordLeft.y * in_projMatLeft.at<float>(2, 3) - in_projMatLeft.at<float>(1, 3);

    A.at<float>(1, 0) = in_cameraCoordLeft.x * in_projMatLeft.at<float>(2, 0) - in_projMatLeft.at<float>(0, 0);
    A.at<float>(1, 1) = in_cameraCoordLeft.x * in_projMatLeft.at<float>(2, 1) - in_projMatLeft.at<float>(0, 1);
    A.at<float>(1, 2) = in_cameraCoordLeft.x * in_projMatLeft.at<float>(2, 2) - in_projMatLeft.at<float>(0, 2);
    A.at<float>(1, 3) = in_cameraCoordLeft.x * in_projMatLeft.at<float>(2, 3) - in_projMatLeft.at<float>(0, 3);

    A.at<float>(2, 0) = in_cameraCoordRight.y * in_projMatRight.at<float>(2, 0) - in_projMatRight.at<float>(1, 0);
    A.at<float>(2, 1) = in_cameraCoordRight.y * in_projMatRight.at<float>(2, 1) - in_projMatRight.at<float>(1, 1);
    A.at<float>(2, 2) = in_cameraCoordRight.y * in_projMatRight.at<float>(2, 2) - in_projMatRight.at<float>(1, 2);
    A.at<float>(2, 3) = in_cameraCoordRight.y * in_projMatRight.at<float>(2, 3) - in_projMatRight.at<float>(1, 3);

    A.at<float>(3, 0) = in_cameraCoordRight.x * in_projMatRight.at<float>(2, 0) - in_projMatRight.at<float>(0, 0);
    A.at<float>(3, 1) = in_cameraCoordRight.x * in_projMatRight.at<float>(2, 1) - in_projMatRight.at<float>(0, 1);
    A.at<float>(3, 2) = in_cameraCoordRight.x * in_projMatRight.at<float>(2, 2) - in_projMatRight.at<float>(0, 2);
    A.at<float>(3, 3) = in_cameraCoordRight.x * in_projMatRight.at<float>(2, 3) - in_projMatRight.at<float>(0, 3);

    // Compute SVD
    cv::Mat U, D, V_t;
    cv::SVDecomp(A, D, U, V_t, cv::SVD::FULL_UV);

    // Solution is given by nullspace of A
    out_triangulatedPoint.x = V_t.row(3).at<float>(0, 0) / V_t.row(3).at<float>(0, 3);
    out_triangulatedPoint.y = V_t.row(3).at<float>(0, 1) / V_t.row(3).at<float>(0, 3);
    out_triangulatedPoint.z = V_t.row(3).at<float>(0, 2) / V_t.row(3).at<float>(0, 3);

    return true;
}


} //namespace Utils
} //namespace VOCPP
