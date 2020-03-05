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

    cv::Mat1f weightedGradxX;
    Utils::ApplyKernelToImage(gradX.mul(gradX), m_smoothingKernel, weightedGradxX);
    cv::Mat1f weightedGradxY;
    Utils::ApplyKernelToImage(gradX.mul(gradY), m_smoothingKernel, weightedGradxY);
    cv::Mat1f weightedGradyY;
    Utils::ApplyKernelToImage(gradY.mul(gradY), m_smoothingKernel, weightedGradyY);

    cv::Mat1f trace = (weightedGradxX + weightedGradxY);

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

} //namespace FeatureHandling
} //namespace VOCPP

