/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/OrFastDetector.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <Vocpp_Utils/TracingImpl.h>

#include<opencv2/imgproc/imgproc.hpp>

namespace VOCPP
{
namespace FeatureHandling
{

// The version of the FAST feature detector used here uses a ring of 16 pixels around the center pixel (radius = 3 --> diameter 7)
const uint32_t OrientedFastDetector::s_featureSize = 7U;

OrientedFastDetector::OrientedFastDetector(const double& in_intDiffTresh, const uint32_t& in_numPixelsAboveThresh,
    const uint32_t& in_harrisBlockSize, const uint32_t& in_distToEdges) :
    m_intDiffTresh(in_intDiffTresh),
    m_numPixelsAboveThresh(in_numPixelsAboveThresh),
    m_harrisBlockSize(in_harrisBlockSize),
    m_distToEdges(in_distToEdges),
    m_harrisDetector()
{
}


bool OrientedFastDetector::ExtractFeatures(const Frame& in_frame, const uint32_t& in_maxNumFeatures, std::vector<Feature>& out_features)
{
    bool ret = true;
    // Reserve memory for maximum number of features
    out_features.reserve(in_maxNumFeatures);

    if (in_frame.GetImage().dims != 2)
    {
        ret = false;
        VOCPP_TRACE_ERROR("[OrientedFastDetector]: Non-grayscale image has been provided")
    }
    else
    {
        cv::Mat1d harrisScore = cv::Mat1d::zeros(in_frame.GetImage().rows, in_frame.GetImage().cols);
        for (int32_t i = m_distToEdges; i < in_frame.GetImage().rows - static_cast<int32_t>(m_distToEdges); i++)
        {
            double* rowPtr = harrisScore.ptr<double>(i);
            for (int32_t j = m_distToEdges; j < in_frame.GetImage().cols - static_cast<int32_t>(m_distToEdges); j++)
            {
                const int32_t score = CheckIntensities(in_frame.GetImage(), j, i, in_frame.GetImage().cols, in_frame.GetImage().rows);
                if (score >= static_cast<int32_t>(m_numPixelsAboveThresh))
                {
                    rowPtr[j] = m_harrisDetector.ComputeScore(in_frame.GetImage(), j, i, m_harrisBlockSize);
                }
            }
        }

        // Thin out neighbouring features, we use such a radius that features do not overlap
        // with respect to the pixels checked by the FAST detector
        std::vector<Utils::LocalMaximum> localMax;
        localMax.reserve(in_maxNumFeatures);
        Utils::ExtractLocalMaxima(harrisScore, (s_featureSize - 1), localMax);

        // Sort according to feature response and throw away weakest features if we have more than we need
        std::sort(localMax.begin(), localMax.end(), std::greater<Utils::LocalMaximum>());
        if (localMax.size() > in_maxNumFeatures)
        {
            localMax.resize(in_maxNumFeatures);
        }
        
        // Calculate orientation for all surviving features
        uint32_t featureId = 0U;
        uint32_t maxId = 0U;
        for (uint32_t idx = 0U; idx < localMax.size(); idx++)
        {
            cv::Mat1d patch = cv::Mat1d::zeros(s_featureSize, s_featureSize);
            bool patchExtractSuccess = Utils::ExtractImagePatchAroundPixelPos(in_frame.GetImage(), patch,  (s_featureSize - 1) / 2,
                static_cast<int32_t>(localMax[idx].posX), static_cast<int32_t>(localMax[idx].posY));
            
            // Calculate angle if successful and create feature
            if (patchExtractSuccess)
            {
                cv::Moments patchMoments = cv::moments(patch);
                const double centroidX = static_cast<double>(patchMoments.m10 / patchMoments.m00);
                const double centroidY = static_cast<double>(patchMoments.m01 / patchMoments.m00);
                const double vecX = centroidX - (s_featureSize - 1) / 2;
                const double vecY = centroidY - (s_featureSize - 1) / 2;
                double angle = static_cast<double>(std::atan2(vecY, vecX));
                Feature feat = { featureId, in_frame.GetId(), localMax[idx].posX, localMax[idx].posY, localMax[idx].value, angle, static_cast<double>(s_featureSize) };
                out_features.push_back(feat);
                featureId++;
            }
            maxId++;
        }
    }
    
    return ret;
}

int32_t OrientedFastDetector::CheckIntensities(const cv::Mat1d& in_image, const int32_t& in_coordX, const int32_t& in_coordY, 
    const int32_t& in_imgWidth, const int32_t& in_imgHeight)
{
    int32_t ret = 0;
    
    const double nucleusInt = in_image(in_coordY, in_coordX);
    // Remove pixels with very small intensity
    if (nucleusInt < (10.0F / 255.0F))
    {
        return 0;
    }
    // Remove pixels with very high intensity
    if (nucleusInt > (250.0F / 255.0F))
    {
        return 0;
    }

    // Pixel 1
    int32_t passLower = (in_image(in_coordY - 3, in_coordX) + m_intDiffTresh) < nucleusInt ? 1 : 0;
    // Pixel 5
    if ((in_image(in_coordY, in_coordX + 3) + m_intDiffTresh) < nucleusInt) passLower++;
    // Pixel 9
    if ((in_image(in_coordY + 3, in_coordX) + m_intDiffTresh) < nucleusInt) passLower++;
    // Pixel 13
    if ((in_image(in_coordY, in_coordX - 3) + m_intDiffTresh) < nucleusInt) passLower++;
    
    if (passLower < 3)
    {
        int32_t passHigher = (in_image(in_coordY - 3, in_coordX) - m_intDiffTresh) > nucleusInt ? 1 : 0;
        // Pixel 5
        if ((in_image(in_coordY, in_coordX + 3) - m_intDiffTresh) > nucleusInt) passHigher++;
        // Pixel 9
        if ((in_image(in_coordY + 3, in_coordX) - m_intDiffTresh) > nucleusInt) passHigher++;
        // Pixel 13
        if ((in_image(in_coordY, in_coordX - 3) - m_intDiffTresh) > nucleusInt) passHigher++;

        if (passHigher >= 3)
        {
            ret = CheckAll(in_image, in_coordX, in_coordY, 0, passHigher);
        }
    }
    else
    {
        ret = CheckAll(in_image, in_coordX, in_coordY, passLower, 0);
    }

    return ret;
}

int32_t OrientedFastDetector::CheckAll(const cv::Mat1d& in_image, const int32_t& in_coordX, const int32_t& in_coordY, const int32_t& in_passLower, const int32_t& in_passHigher)
{
    const double nucleusInt = in_image(in_coordY, in_coordX);
    int32_t score = 0;
    
    if (in_passLower > 0)
    {
        score = in_passLower;
        // Pixel 2
        if ((in_image(in_coordY - 3, in_coordX + 1) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 3
        if ((in_image(in_coordY - 2, in_coordX + 2) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 4
        if ((in_image(in_coordY - 1, in_coordX + 3) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 6
        if ((in_image(in_coordY + 1, in_coordX + 3) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 7
        if ((in_image(in_coordY + 2, in_coordX + 2) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 8
        if ((in_image(in_coordY + 3, in_coordX + 1) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 10
        if ((in_image(in_coordY + 3, in_coordX - 1) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 11
        if ((in_image(in_coordY + 2, in_coordX - 2) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 12
        if ((in_image(in_coordY + 1, in_coordX - 3) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 14
        if ((in_image(in_coordY - 1, in_coordX - 3) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 15
        if ((in_image(in_coordY - 2, in_coordX - 2) + m_intDiffTresh) < nucleusInt) score++;
        // Pixel 16
        if ((in_image(in_coordY - 3, in_coordX - 1) + m_intDiffTresh) < nucleusInt) score++;
    }
    else if(in_passHigher > 0)
    {
        score = in_passHigher;
        // Pixel 2
        if ((in_image(in_coordY - 3, in_coordX + 1) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 3
        if ((in_image(in_coordY - 2, in_coordX + 2) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 4
        if ((in_image(in_coordY - 1, in_coordX + 3) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 6
        if ((in_image(in_coordY + 1, in_coordX + 3) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 7
        if ((in_image(in_coordY + 2, in_coordX + 2) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 8
        if ((in_image(in_coordY + 3, in_coordX + 1) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 10
        if ((in_image(in_coordY + 3, in_coordX - 1) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 11
        if ((in_image(in_coordY + 2, in_coordX - 2) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 12
        if ((in_image(in_coordY + 1, in_coordX - 3) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 14
        if ((in_image(in_coordY - 1, in_coordX - 3) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 15
        if ((in_image(in_coordY - 2, in_coordX - 2) - m_intDiffTresh) > nucleusInt) score++;
        // Pixel 16
        if ((in_image(in_coordY - 3, in_coordX - 1) - m_intDiffTresh) > nucleusInt) score++;
    }
    
    return score;
}

} //namespace FeatureHandling
} //namespace VOCPP

