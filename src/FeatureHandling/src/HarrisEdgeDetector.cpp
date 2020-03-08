/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/HarrisEdgeDetector.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <iostream>

namespace VOCPP
{
namespace FeatureHandling
{


HarrisEdgeDetector::HarrisEdgeDetector(const int in_maxNumFeatures, const float in_k, const std::string& in_kernelName,
    const int in_localMaxDistance, const int in_subPixelCalculationDistance)
{
    m_maxNumFeatures = in_maxNumFeatures;
    m_k = in_k;
    m_localMaxDistance = in_localMaxDistance;
    m_subPixelCalculationDistance = in_subPixelCalculationDistance;
    m_smoothingKernel = Utils::GetWindowKernel(3);
}


bool HarrisEdgeDetector::ExtractFeatures(const Frame& in_frame, std::vector<Feature>& out_features)
{
    bool ret = true;

    if (in_frame.GetImage().dims != 2)
    {
        ret = false;
        std::cout << "[HarrisEdgeDetector]: Non-grayscale image has been provided" << std::endl;
    }
    
    cv::Mat1f gradX, gradY;
    Utils::Compute2DGradients(in_frame.GetImage(), gradX, gradY);
    
    cv::Mat1f newResponse = cv::Mat1f::zeros(in_frame.GetImage().rows, in_frame.GetImage().cols);
    for (int i = 1; i < in_frame.GetImage().cols - 1; i++)
    {
        for (int j = 1; j < in_frame.GetImage().rows - 1; j++)
        {
            newResponse(j,i) = ComputeScore(in_frame.GetImage(), i, j, 3);
        }
    }

    cv::Mat1f weightedGradxX;
    Utils::ApplyKernelToImage(gradX.mul(gradX), m_smoothingKernel, weightedGradxX);
    cv::Mat1f weightedGradxY;
    Utils::ApplyKernelToImage(gradX.mul(gradY), m_smoothingKernel, weightedGradxY);
    cv::Mat1f weightedGradyY;
    Utils::ApplyKernelToImage(gradY.mul(gradY), m_smoothingKernel, weightedGradyY);

    cv::Mat1f trace = (weightedGradxX + weightedGradyY);

    cv::Mat1f response = (weightedGradxX.mul(weightedGradyY) - weightedGradxY.mul(weightedGradxY))
        - ((trace).mul(trace) * m_k);

    // Extract local maxima
    std::vector<cv::Point2f> localMax;
    std::vector<float> localMaxVal;
    Utils::ExtractLocalMaxima(response, m_localMaxDistance, localMax, localMaxVal,m_subPixelCalculationDistance);

    int featureId = 0;
    for (auto point : localMax)
    {
        out_features.push_back(Feature{ featureId, in_frame.GetId(), point.x, point.y, localMaxVal[featureId] });
        featureId++;
    }

    // Sort according to feature response and throw away weakest features if we have more than we need
    std::sort(out_features.begin(), out_features.end(), std::greater<Feature>());
    if (out_features.size() > m_maxNumFeatures)
    {
        out_features.resize(m_maxNumFeatures);
    }

    return ret;
}

float HarrisEdgeDetector::ComputeScore(const cv::Mat1f& in_img, const int& in_centerX, const int& in_centerY, const int& in_blockSize)
{
    // block size has to be an odd number
    int distanceFromCenter = (in_blockSize - 1) / 2;
    // Check range, +1 is necessary since the Sobel Kernel is 3x3
    bool inRangeX = in_centerX - distanceFromCenter - 1 >= 0 && in_centerX + distanceFromCenter + 1 < in_img.cols;
    bool inRangeY = in_centerY - distanceFromCenter - 1 >= 0 && in_centerY + distanceFromCenter + 1 < in_img.rows;

    float response = -1.0F;

    if (inRangeX && inRangeY)
    {
        float gradXx = 0;
        float gradYy = 0;
        float gradXy = 0;
        
        for (int i = in_centerX - distanceFromCenter; i < in_centerX + distanceFromCenter + 1; i++)
        {
            for (int j = in_centerY - distanceFromCenter; j < in_centerY + distanceFromCenter + 1; j++)
            {
                float gradX = in_img(j - 1, i + 1) - in_img(j - 1, i - 1) + 2.0F * (in_img(j, i + 1) - in_img(j, i - 1)) + in_img(j + 1, i + 1) - in_img(j + 1, i - 1);
                float gradY = in_img(j + 1, i - 1) - in_img(j - 1, i - 1) + 2.0F * (in_img(j + 1, i) - in_img(j - 1, i)) + in_img(j + 1, i + 1) - in_img(j -1, i + 1);
                
                gradXx += gradX * gradX;
                gradYy += gradY * gradY;
                gradXy += gradX * gradY;
            }
        }

        const float trace = gradXx + gradYy;
        response = gradXx * gradYy - std::pow(gradXy, 2) - m_k * std::pow(trace, 2);
    }

    return response;
}

} //namespace FeatureHandling
} //namespace VOCPP

