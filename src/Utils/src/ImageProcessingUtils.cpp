/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/ImageProcessingUtils.h>
#include<opencv2/imgproc.hpp>
#include<iostream>

namespace VOCPP
{
namespace Utils
{

cv::Mat GetWindowKernel(const uint32_t size)
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

void ExtractLocalMaxima(const cv::Mat& in_image, const uint32_t in_distance, std::vector<cv::Point2f>& out_localMaxima, const uint32_t in_subPixelCalculationDistance)
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
            const uint32_t patchDim = 2 * in_subPixelCalculationDistance + 1U;
            cv::Mat patch = cv::Mat::zeros(patchDim, patchDim, CV_32F);
            bool subPixSuccess = ExtractImagePatchAroundPixelPos(in_image, patch, in_subPixelCalculationDistance, static_cast<uint32_t>(pos->x), static_cast<uint32_t>(pos->y));

            if (subPixSuccess)
            {
                cv::Moments patchMoments = cv::moments(patch);

                pos->x = pos->x - in_subPixelCalculationDistance + static_cast<float>(patchMoments.m10 / patchMoments.m00);
                pos->y = pos->y - in_subPixelCalculationDistance + static_cast<float>(patchMoments.m01 / patchMoments.m00);
            }
        }
    }

}

bool ExtractImagePatchAroundPixelPos(const cv::Mat& in_image, cv::Mat& out_patch, const uint32_t in_distanceAroundCenter, const uint32_t in_pixelPosX, const uint32_t in_pixelPosY)
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

    const float scale = std::sqrt(2) / std::sqrt(meanDist);

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

bool GetCrossProductMatrix(const cv::Mat& in_generatingMat, cv::Mat& out_crossMat)
{
    bool ret = true;

    if (in_generatingMat.rows != 3 || in_generatingMat.cols != 1)
    {
        std::cout << "[GetCrossProductMatrix]: cross product generating vector has to be 3x1" << std::endl;
        ret = false;
    }
    else
    {
        out_crossMat = cv::Mat::zeros(3, 3, CV_32F);
        out_crossMat.at<float>(0, 1) = -in_generatingMat.at<float>(2, 0);
        out_crossMat.at<float>(0, 2) = in_generatingMat.at<float>(1, 0);
        out_crossMat.at<float>(1, 0) = in_generatingMat.at<float>(2, 0);
        out_crossMat.at<float>(1, 2) = -in_generatingMat.at<float>(0, 0);
        out_crossMat.at<float>(2, 0) = -in_generatingMat.at<float>(1, 0);
        out_crossMat.at<float>(2, 1) = in_generatingMat.at<float>(0, 0);
    }

    return ret;
}


} //namespace Utils
} //namespace VOCPP
