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

    const int32_t patchSize = 1;
    for (int32_t i = 0; i < in_image.size[1]; i++)
    {
        for (int32_t j = 0; j < in_image.size[0]; j++)
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
    const bool inRangeX = static_cast<int32_t>(in_pixelPosX - in_distanceAroundCenter) >= 0 && static_cast<int32_t>(in_pixelPosX + in_distanceAroundCenter + 1U) < in_image.size[1];
    const bool inRangeY = static_cast<int32_t>(in_pixelPosY - in_distanceAroundCenter) >= 0 && static_cast<int32_t>(in_pixelPosY + in_distanceAroundCenter + 1U) < in_image.size[0];
    // Then check if dimensions of provided patch matrix are correct
    const bool correctPatchDimX = out_patch.size[1] == (2 * in_distanceAroundCenter + 1);
    const bool correctPatchDimY = out_patch.size[0] == (2 * in_distanceAroundCenter + 1);

    if (inRangeX && inRangeY && correctPatchDimX && correctPatchDimY)
    {
        for (int32_t i = 0; i < out_patch.size[1]; i++)
        {
            for (int32_t j = 0; j < out_patch.size[0]; j++)
            {
                int32_t imagePosX = in_pixelPosX - in_distanceAroundCenter + i;
                int32_t imagePosÝ = in_pixelPosY - in_distanceAroundCenter + j;
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

} //namespace Utils
} //namespace VOCPP
