/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include<HarrisEdgeDetector.h>
#include<Vocpp_Utils/ImageProcessingUtils.h>
#include <opencv2/imgproc.hpp>
#include <iostream>

namespace VOCPP
{
namespace FeatureHandling
{

HarrisEdgeDetector::HarrisEdgeDetector()
{
}


HarrisEdgeDetector* HarrisEdgeDetector::CreateInstance(const double in_relResponseThresh, const double in_k, const std::string& in_kernelName,
    const int in_localMaxDistance, const int in_subPixelCalculationDistance)
{
    HarrisEdgeDetector* det = new HarrisEdgeDetector();

    det->m_relResponseThresh = in_relResponseThresh;
    det->m_k = in_k;
    det->m_localMaxDistance = in_localMaxDistance;
    det->m_subPixelCalculationDistance = in_subPixelCalculationDistance;

    if (in_kernelName == "window")
    {
        det->m_smoothingKernel = Utils::GetWindowKernel(3);
    }
    else
    {
        std::cout << "[HarrisEdgeDetector]: Specified unknown smoothing kernel" << std::endl;
        delete det;
        det = NULL;
    }

    return det;
}


bool HarrisEdgeDetector::ExtractKeypoints(const Frame& in_frame, std::vector<cv::KeyPoint>& out_keypoints)
{
    bool ret = true;

    if (in_frame.GetImage().dims != 2)
    {
        ret = false;
        std::cout << "[HarrisEdgeDetector]: Non-grayscale image has been provided" << std::endl;
    }
    
    cv::Mat gradX, gradY;
    Utils::Compute2DGradients(in_frame.GetImage(), gradX, gradY);

    cv::Mat weightedGradxX;
    Utils::ApplyKernelToImage(gradX.mul(gradX), m_smoothingKernel, weightedGradxX);
    cv::Mat weightedGradxY;
    Utils::ApplyKernelToImage(gradX.mul(gradY), m_smoothingKernel, weightedGradxY);
    cv::Mat weightedGradyY;
    Utils::ApplyKernelToImage(gradY.mul(gradY), m_smoothingKernel, weightedGradyY);

    cv::Mat trace = (weightedGradxX + weightedGradxY);

    cv::Mat response = (weightedGradxX.mul(weightedGradyY) - weightedGradxY.mul(weightedGradxY))
        - ((trace).mul(trace) * m_k);

    // Define response threshold using the relative response value    
    double maxResponse = 0;
    cv::minMaxLoc(response, NULL, &maxResponse);
    const double thresh = maxResponse * m_relResponseThresh;
    cv::threshold(response, response, thresh, maxResponse, cv::THRESH_TOZERO);

    // Extract local maxima
    std::vector<cv::Point2f> localMax;
    Utils::ExtractLocalMaxima(response, m_localMaxDistance, localMax, m_subPixelCalculationDistance);

    for (auto point : localMax)
    {
        out_keypoints.push_back(cv::KeyPoint(point, static_cast<float>(m_localMaxDistance)));
    }
    
    return ret;

}

} //namespace FeatureHandling
} //namespace VOCPP

