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

cv::Mat1f GetWindowKernel(const int size)
{
    return cv::Mat1f::ones(size, size);
}

void ApplyKernelToImage(const cv::Mat1f& in_image, const cv::Mat1f& in_kernel, cv::Mat1f& out_image)
{
    cv::filter2D(in_image, out_image, CV_32F, in_kernel);
}

void Compute2DGradients(const cv::Mat1f& in_image, cv::Mat1f& out_gradX, cv::Mat1f& out_gradY)
{ 
    // Compute gradient using sobel operators
    // Sobel operator can be replaced by two convolutions
    float dataRowX[3] = { -1.0, 0.0, 1.0 };
    cv::Mat1f sobelRowX = cv::Mat1f(1, 3, dataRowX);
    float dataColX[3] = { 1.0, 2.0, 1.0 };
    cv::Mat1f sobelColX = cv::Mat1f(3, 1, dataColX);
    cv::sepFilter2D(in_image, out_gradX, CV_32F, sobelRowX, sobelColX);

    float dataRowY[3] = { 1.0, 2.0, 1.0 };
    cv::Mat1f sobelRowY = cv::Mat1f(1, 3, dataRowY);
    float dataColY[3] = { -1.0, 0.0, 1.0 };
    cv::Mat1f sobelColY = cv::Mat1f(3, 1, dataColY);
    cv::sepFilter2D(in_image, out_gradY, CV_32F, sobelRowY, sobelColY);
}

void ExtractLocalMaxima(const cv::Mat1f& in_image, const int in_distance, std::vector<cv::Point2f>& out_localMaxima, 
    std::vector<float>& out_localMaximaValue, const int in_subPixelCalculationDistance)
{
    cv::Mat1f dilatedImage;
    cv::Mat1f kernel = GetWindowKernel(2 * in_distance + 1);
    cv::dilate(in_image, dilatedImage, kernel);

    for (int i = 0; i < in_image.size[1]; i++)
    {
        for (int j = 0; j < in_image.size[0]; j++)
        {
            bool greaterZero = in_image(j, i) > 0.0;
            // If image value is equal to the dilated value (maximum value of surrounding pixels)
            // this pixel has the maximum value
            if (greaterZero && in_image(j, i) == dilatedImage(j, i))
            {
                out_localMaxima.push_back(cv::Point2f(static_cast<float>(i), static_cast<float>(j)));
                out_localMaximaValue.push_back(in_image(j, i));
            }
        }
    }

    // If specified, try to calculate sub-pixel position by looking at centroids
    if (in_subPixelCalculationDistance > 0U)
    {
        for (auto pos = out_localMaxima.begin(); pos != out_localMaxima.end(); pos++)
        {
            const int patchDim = 2 * in_subPixelCalculationDistance + 1U;
            cv::Mat1f patch = cv::Mat1f::zeros(patchDim, patchDim);
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

bool ExtractImagePatchAroundPixelPos(const cv::Mat1f& in_image, cv::Mat1f& out_patch, const int in_distanceAroundCenter, const int in_pixelPosX, const int in_pixelPosY)
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
                int imagePosY = in_pixelPosY - in_distanceAroundCenter + j;
                out_patch(j, i) = in_image(in_pixelPosY - in_distanceAroundCenter + j, in_pixelPosX - in_distanceAroundCenter + i);
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

void NormalizePointSet(const std::vector<cv::Point2f>& in_points, std::vector<cv::Point2f>& out_normPoints, cv::Mat1f& out_transform)
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

    out_transform = cv::Mat1f::zeros(3, 3);
    out_transform(0, 0) = scale;
    out_transform(1, 1) = scale;
    out_transform(0, 2) = -scale * meanX;
    out_transform(1, 2) = -scale * meanY;
    out_transform(2, 2) = 1.0;
}

bool GetProjectionMatrix(const cv::Mat1f& in_rotationMatrix, const cv::Mat1f& in_translation, cv::Mat1f& out_projMat)
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

void GetCrossProductMatrix(const cv::Vec3f& in_vec, cv::Mat1f& out_crossMat)
{
    out_crossMat = cv::Mat1f::zeros(3, 3);
    out_crossMat(0, 1) = -in_vec[2];
    out_crossMat(0, 2) = in_vec[1];
    out_crossMat(1, 0) = in_vec[2];
    out_crossMat(1, 2) = -in_vec[0];
    out_crossMat(2, 0) = -in_vec[1];
    out_crossMat(2, 1) = in_vec[0];
}

bool DecomposeEssentialMatrix(const cv::Mat1f& in_essentialMat, const cv::Point2f& in_matchPointLeft,
    const cv::Point2f& in_matchPointRight, cv::Mat1f& out_translation, cv::Mat1f& out_rotMatrix)
{
    // Compute SVD
    cv::Mat1f U, D, V_t;
    cv::SVDecomp(in_essentialMat, D, U, V_t, cv::SVD::FULL_UV);
   
    // Set smallest singular value to zero and recompute SVD
    cv::Mat1f newDiag = cv::Mat1f::zeros(3, 3);
    newDiag(0, 0) = 1.0;
    newDiag(1, 1) = 1.0;

    cv::Mat1f newEssentialMat = U * (newDiag * V_t);
    cv::SVDecomp(newEssentialMat, D, U, V_t, cv::SVD::FULL_UV);
    if (cv::determinant(U * V_t) < 0.0)
    {
        V_t = -V_t;
    }

    cv::Mat1f Y = cv::Mat1f::zeros(3, 3);
    Y(0, 1) = -1.0;
    Y(1, 0) = 1.0;
    Y(2, 2) = 1.0;

    // Two solutions for translation
    cv::Mat1f t1 = U.col(2);
    cv::Mat1f t2 = -t1;

    // Two solutions for rotation matrix
    cv::Mat1f R1 = U * (Y * V_t);
    cv::Mat1f R2 = U * (Y.t() * V_t);

    // Right projection matrix is trivial one
    cv::Mat1f projMatRight;
    GetProjectionMatrix(cv::Mat1f::eye(3, 3), cv::Mat1f::zeros(3, 1), projMatRight);

    // Four different projection matrices for right one
    std::vector<cv::Mat1f> projectionMatCandidates;

    cv::Mat1f projMatLeft1;
    cv::Mat1f projMatLeft2;
    GetProjectionMatrix(R1, t1, projMatLeft1);
    GetProjectionMatrix(R1, t2, projMatLeft2);
    projectionMatCandidates.push_back(projMatLeft1);
    projectionMatCandidates.push_back(projMatLeft2);
 
    cv::Mat1f projMatLeft3;
    cv::Mat1f projMatLeft4;
    GetProjectionMatrix(R2, t1, projMatLeft3);
    GetProjectionMatrix(R2, t2, projMatLeft4);
    projectionMatCandidates.push_back(projMatLeft3);
    projectionMatCandidates.push_back(projMatLeft4);

    // Check which of the solutions is the correct one by demanding that the triangulated point is in front of both cameras
    for (auto candidate : projectionMatCandidates)
    {
        cv::Point3f triangPoint;
        PointTriangulationLinear(candidate, projMatRight, in_matchPointLeft, in_matchPointRight, triangPoint);
        cv::Mat1f projPointLeft = candidate * VOCPP::Utils::Point3fToMatHomCoordinates(triangPoint);
        if (triangPoint.z > 0.0 && projPointLeft(2,0) > 0)
        {
            candidate(cv::Range(0, 3), cv::Range(0, 3)).copyTo(out_rotMatrix);
            candidate(cv::Range(0, 3), cv::Range(3, 4)).copyTo(out_translation);
        }
    }
  

    return false;
}

bool PointTriangulationLinear(const cv::Mat1f& in_projMatLeft, const cv::Mat1f& in_projMatRight, const cv::Point2f& in_cameraCoordLeft,
    const cv::Point2f& in_cameraCoordRight, cv::Point3f& out_triangulatedPoint)
{
    cv::Mat1f A = cv::Mat1f::zeros(4, 4);

    // Construct matrix to decompose (see for example https://www.uio.no/studier/emner/matnat/its/nedlagte-emner/UNIK4690/v16/forelesninger/lecture_7_2-triangulation.pdf)
    A(0, 0) = in_cameraCoordLeft.y * in_projMatLeft(2, 0) - in_projMatLeft(1, 0);
    A(0, 1) = in_cameraCoordLeft.y * in_projMatLeft(2, 1) - in_projMatLeft(1, 1);
    A(0, 2) = in_cameraCoordLeft.y * in_projMatLeft(2, 2) - in_projMatLeft(1, 2);
    A(0, 3) = in_cameraCoordLeft.y * in_projMatLeft(2, 3) - in_projMatLeft(1, 3);

    A(1, 0) = in_cameraCoordLeft.x * in_projMatLeft(2, 0) - in_projMatLeft(0, 0);
    A(1, 1) = in_cameraCoordLeft.x * in_projMatLeft(2, 1) - in_projMatLeft(0, 1);
    A(1, 2) = in_cameraCoordLeft.x * in_projMatLeft(2, 2) - in_projMatLeft(0, 2);
    A(1, 3) = in_cameraCoordLeft.x * in_projMatLeft(2, 3) - in_projMatLeft(0, 3);

    A(2, 0) = in_cameraCoordRight.y * in_projMatRight(2, 0) - in_projMatRight(1, 0);
    A(2, 1) = in_cameraCoordRight.y * in_projMatRight(2, 1) - in_projMatRight(1, 1);
    A(2, 2) = in_cameraCoordRight.y * in_projMatRight(2, 2) - in_projMatRight(1, 2);
    A(2, 3) = in_cameraCoordRight.y * in_projMatRight(2, 3) - in_projMatRight(1, 3);

    A(3, 0) = in_cameraCoordRight.x * in_projMatRight(2, 0) - in_projMatRight(0, 0);
    A(3, 1) = in_cameraCoordRight.x * in_projMatRight(2, 1) - in_projMatRight(0, 1);
    A(3, 2) = in_cameraCoordRight.x * in_projMatRight(2, 2) - in_projMatRight(0, 2);
    A(3, 3) = in_cameraCoordRight.x * in_projMatRight(2, 3) - in_projMatRight(0, 3);

    // Compute SVD
    cv::Mat1f U, D, V_t;
    cv::SVDecomp(A, D, U, V_t, cv::SVD::FULL_UV);

    // Solution is given by nullspace of A
    out_triangulatedPoint.x = V_t.row(3)(0, 0) / V_t.row(3)(0, 3);
    out_triangulatedPoint.y = V_t.row(3)(0, 1) / V_t.row(3)(0, 3);
    out_triangulatedPoint.z = V_t.row(3)(0, 2) / V_t.row(3)(0, 3);

    return true;
}


} //namespace Utils
} //namespace VOCPP
