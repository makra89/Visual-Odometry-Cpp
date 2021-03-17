/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2021 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/HarrisEdgeDetector.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <Vocpp_Utils/TracingImpl.h>

namespace VOCPP
{
namespace FeatureHandling
{


HarrisEdgeDetector::HarrisEdgeDetector(const uint32_t in_maxNumFeatures, const double in_k, const std::string& in_kernelName,
    const uint32_t in_localMaxDistance)
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
        VOCPP_TRACE_ERROR("[HarrisEdgeDetector]: Non-grayscale image has been provided")
    }
    
    cv::Mat1d gradX, gradY;
    Utils::Compute2DGradients(in_frame.GetImage(), gradX, gradY);
    
    cv::Mat1d response = cv::Mat1d::zeros(in_frame.GetImage().rows, in_frame.GetImage().cols);
    for (int32_t i = 1; i < in_frame.GetImage().cols - 1; i++)
    {
        for (int32_t j = 1; j < in_frame.GetImage().rows - 1; j++)
        {
            response(j,i) = ComputeScore(in_frame.GetImage(), i, j, 3);
        }
    }

    // Extract local maxima
    std::vector<Utils::LocalMaximum> localMax;
    Utils::ExtractLocalMaxima(response, m_localMaxDistance, localMax);

    uint32_t featureId = 0U;
    for (uint32_t idx = 0U; idx < localMax.size(); idx++)
    {
        Feature feat = { featureId, in_frame.GetId(), localMax[idx].posX, localMax[idx].posY, localMax[idx].value };
        out_features.push_back(feat);
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

double HarrisEdgeDetector::ComputeScore(const cv::Mat1d& in_img, const int32_t& in_centerX, const int32_t& in_centerY, const int32_t& in_blockSize)
{
    // block size has to be an odd number
    int32_t distanceFromCenter = (in_blockSize - 1) / 2;
    // Check range, +1 is necessary since the Sobel Kernel is 3x3
    bool inRangeX = in_centerX - distanceFromCenter - 1 >= 0 && in_centerX + distanceFromCenter + 1 < in_img.cols;
    bool inRangeY = in_centerY - distanceFromCenter - 1 >= 0 && in_centerY + distanceFromCenter + 1 < in_img.rows;

    double response = -1.0F;
    
    if (inRangeX && inRangeY)
    {
        double gradXx = 0;
        double gradYy = 0;
        double gradXy = 0;
        
        for (int32_t j = in_centerY - distanceFromCenter; j < in_centerY + distanceFromCenter + 1; j++)
        {
            const double* currRowPtr = in_img.ptr<double>(j);
            const double* upRowPtr = in_img.ptr<double>(j - 1);
            const double* lowRowPtr = in_img.ptr<double>(j + 1);

            for (int32_t i = in_centerX - distanceFromCenter; i < in_centerX + distanceFromCenter + 1; i++)
            {
                double gradX = upRowPtr[i + 1] - upRowPtr[i - 1] + 2.0F * (currRowPtr[i + 1] - currRowPtr[i - 1]) + lowRowPtr[i + 1] - lowRowPtr[i - 1];
                double gradY = lowRowPtr[i - 1] - upRowPtr[i - 1] + 2.0F * (lowRowPtr[i] - upRowPtr[i]) + lowRowPtr[i + 1] - upRowPtr[i + 1];
                
                gradXx += gradX * gradX;
                gradYy += gradY * gradY;
                gradXy += gradX * gradY;
            }
        }

        const double trace = gradXx + gradYy;
        response = gradXx * gradYy - std::pow(gradXy, 2) - m_k * std::pow(trace, 2);
    }

    return response;
}

} //namespace FeatureHandling
} //namespace VOCPP

