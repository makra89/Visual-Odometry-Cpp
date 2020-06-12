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


HarrisEdgeDetector::HarrisEdgeDetector(const unsigned int in_maxNumFeatures, const float in_k, const std::string& in_kernelName,
    const int in_localMaxDistance)
{
    m_maxNumFeatures = in_maxNumFeatures;
    m_k = in_k;
    m_localMaxDistance = in_localMaxDistance;
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
    
    cv::Mat1f response = cv::Mat1f::zeros(in_frame.GetImage().rows, in_frame.GetImage().cols);
    for (int i = 1; i < in_frame.GetImage().cols - 1; i++)
    {
        for (int j = 1; j < in_frame.GetImage().rows - 1; j++)
        {
            response(j,i) = ComputeScore(in_frame.GetImage(), i, j, 3);
        }
    }

    // Extract local maxima
    std::vector<Utils::LocalMaximum> localMax;
    Utils::ExtractLocalMaxima(response, m_localMaxDistance, localMax);

    unsigned int featureId = 0U;
    for (auto max : localMax)
    {
        out_features.push_back(Feature{ featureId, in_frame.GetId(), max.posX, max.posY, max.value});
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
        
        for (int j = in_centerY - distanceFromCenter; j < in_centerY + distanceFromCenter + 1; j++)
        {
            const float* currRowPtr = in_img.ptr<float>(j);
            const float* upRowPtr = in_img.ptr<float>(j - 1);
            const float* lowRowPtr = in_img.ptr<float>(j + 1);

            for (int i = in_centerX - distanceFromCenter; i < in_centerX + distanceFromCenter + 1; i++)
            {
                float gradX = upRowPtr[i + 1] - upRowPtr[i - 1] + 2.0F * (currRowPtr[i + 1] - currRowPtr[i - 1]) + lowRowPtr[i + 1] - lowRowPtr[i - 1];
                float gradY = lowRowPtr[i - 1] - upRowPtr[i - 1] + 2.0F * (lowRowPtr[i] - upRowPtr[i]) + lowRowPtr[i + 1] - upRowPtr[i + 1];
                
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

