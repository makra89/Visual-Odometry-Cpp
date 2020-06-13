/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/OrFastDetector.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include<opencv2/imgproc.hpp>

namespace VOCPP
{
namespace FeatureHandling
{

// The version of the FAST feature detector used here uses a ring of 16 pixels around the center pixel (radius = 3 --> diameter 7)
const int OrientedFastDetector::s_featureSize = 7;

OrientedFastDetector::OrientedFastDetector(const float& in_intDiffTresh, const int& in_numPixelsAboveThresh,
    const int& in_harrisBlockSize, const int& in_distToEdges) :
    m_relDiffTresh(in_intDiffTresh),
    m_numPixelsAboveThresh(in_numPixelsAboveThresh),
    m_harrisBlockSize(in_harrisBlockSize),
    m_distToEdges(in_distToEdges),
    m_harrisDetector()
{
}


bool OrientedFastDetector::ExtractFeatures(const Frame& in_frame, const unsigned int& in_maxNumFeatures, std::vector<Feature>& out_features)
{
    bool ret = true;
    // Reserve memory for maximum number of features
    out_features.reserve(in_maxNumFeatures);

    if (in_frame.GetImage().dims != 2)
    {
        ret = false;
        std::cout << "[OrientedFastDetector]: Non-grayscale image has been provided" << std::endl;
    }
    else
    {
        cv::Mat1f harrisScore = cv::Mat1f::zeros(in_frame.GetImage().rows, in_frame.GetImage().cols);
        for (int i = m_distToEdges; i < in_frame.GetImage().rows - m_distToEdges; i++)
        {
            float* rowPtr = harrisScore.ptr<float>(i);
            for (int j = m_distToEdges; j < in_frame.GetImage().cols - m_distToEdges; j++)
            {
                const int score = CheckIntensities(in_frame.GetImage(), j, i, in_frame.GetImage().cols, in_frame.GetImage().rows);
                if (score >= m_numPixelsAboveThresh)
                {
                    rowPtr[j] = m_harrisDetector.ComputeScore(in_frame.GetImage(), j, i, m_harrisBlockSize);
                }
            }
        }

        // Thin out neighbouring features, we use the same radius as the one used for calculating the angle
        std::vector<Utils::LocalMaximum> localMax;
        localMax.reserve(in_maxNumFeatures);
        Utils::ExtractLocalMaxima(harrisScore, (s_featureSize - 1) / 2, localMax);

        // Sort according to feature response and throw away weakest features if we have more than we need
        std::sort(localMax.begin(), localMax.end(), std::greater<Utils::LocalMaximum>());
        if (localMax.size() > in_maxNumFeatures)
        {
            localMax.resize(in_maxNumFeatures);
        }
        
        // Calculate Harris response for all surviving features
        unsigned int featureId = 0U;
        unsigned int maxId = 0U;
        for (auto max : localMax)
        {
            cv::Mat1f patch = cv::Mat1f::zeros(s_featureSize, s_featureSize);
            bool patchExtractSuccess = Utils::ExtractImagePatchAroundPixelPos(in_frame.GetImage(), patch,  (s_featureSize - 1) / 2,
                static_cast<int>(max.posX), static_cast<int>(max.posY));
            
            // Calculate angle if successful and create feature
            if (patchExtractSuccess)
            {
                cv::Moments patchMoments = cv::moments(patch);
                const float centroidX = static_cast<float>(patchMoments.m10 / patchMoments.m00);
                const float centroidY = static_cast<float>(patchMoments.m01 / patchMoments.m00);
                const float vecX = centroidX - (s_featureSize - 1) / 2;
                const float vecY = centroidY - (s_featureSize - 1) / 2;
                float angle = static_cast<float>(std::atan2(vecY, vecX));
                out_features.push_back(Feature{ featureId, in_frame.GetId(), max.posX, max.posY, max.value, angle, static_cast<float>(s_featureSize)});
                featureId++;
            }
            maxId++;
        }
    }
    
    return ret;
}

int OrientedFastDetector::CheckIntensities(const cv::Mat1f& in_image, const int& in_coordX, const int& in_coordY, 
    const int& in_imgWidth, const int& in_imgHeight)
{
    int ret = 0;
    
    const float nucleusInt = in_image(in_coordY, in_coordX);
    // Remove pixels with very small intensity
    if (nucleusInt < 10.0F / 255.0F)
    {
        return 0;
    }

    // Pixel 1
    int passLower = (in_image(in_coordY - 3, in_coordX) + (m_relDiffTresh * nucleusInt)) < nucleusInt ? 1 : 0;
    // Pixel 5
    if ((in_image(in_coordY, in_coordX + 3) + (m_relDiffTresh * nucleusInt)) < nucleusInt) passLower++;
    // Pixel 9
    if ((in_image(in_coordY + 3, in_coordX) + (m_relDiffTresh * nucleusInt)) < nucleusInt) passLower++;
    // Pixel 13
    if ((in_image(in_coordY, in_coordX - 3) + (m_relDiffTresh * nucleusInt)) < nucleusInt) passLower++;
    
    if (passLower < 3)
    {
        int passHigher = (in_image(in_coordY - 3, in_coordX) - (m_relDiffTresh * nucleusInt)) > nucleusInt ? 1 : 0;
        // Pixel 5
        if ((in_image(in_coordY, in_coordX + 3) - (m_relDiffTresh * nucleusInt)) > nucleusInt) passHigher++;
        // Pixel 9
        if ((in_image(in_coordY + 3, in_coordX) - (m_relDiffTresh * nucleusInt)) > nucleusInt) passHigher++;
        // Pixel 13
        if ((in_image(in_coordY, in_coordX - 3) - (m_relDiffTresh * nucleusInt)) > nucleusInt) passHigher++;

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

int OrientedFastDetector::CheckAll(const cv::Mat1f& in_image, const int& in_coordX, const int& in_coordY, const int& in_passLower, const int& in_passHigher)
{
    const float nucleusInt = in_image(in_coordY, in_coordX);
    int score = 0;
    
    if (in_passLower > 0)
    {
        score = in_passLower;
        // Pixel 2
        if ((in_image(in_coordY - 3, in_coordX + 1) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 3
        if ((in_image(in_coordY - 2, in_coordX + 2) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 4
        if ((in_image(in_coordY - 1, in_coordX + 3) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 6
        if ((in_image(in_coordY + 1, in_coordX + 3) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 7
        if ((in_image(in_coordY + 2, in_coordX + 2) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 8
        if ((in_image(in_coordY + 3, in_coordX + 1) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 10
        if ((in_image(in_coordY + 3, in_coordX - 1) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 11
        if ((in_image(in_coordY + 2, in_coordX - 2) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 12
        if ((in_image(in_coordY + 1, in_coordX - 3) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 14
        if ((in_image(in_coordY - 1, in_coordX - 3) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 15
        if ((in_image(in_coordY - 2, in_coordX - 2) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
        // Pixel 16
        if ((in_image(in_coordY - 3, in_coordX - 1) + (m_relDiffTresh * nucleusInt)) < nucleusInt) score++;
    }
    else if(in_passHigher > 0)
    {
        score = in_passHigher;
        // Pixel 2
        if ((in_image(in_coordY - 3, in_coordX + 1) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 3
        if ((in_image(in_coordY - 2, in_coordX + 2) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 4
        if ((in_image(in_coordY - 1, in_coordX + 3) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 6
        if ((in_image(in_coordY + 1, in_coordX + 3) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 7
        if ((in_image(in_coordY + 2, in_coordX + 2) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 8
        if ((in_image(in_coordY + 3, in_coordX + 1) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 10
        if ((in_image(in_coordY + 3, in_coordX - 1) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 11
        if ((in_image(in_coordY + 2, in_coordX - 2) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 12
        if ((in_image(in_coordY + 1, in_coordX - 3) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 14
        if ((in_image(in_coordY - 1, in_coordX - 3) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 15
        if ((in_image(in_coordY - 2, in_coordX - 2) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
        // Pixel 16
        if ((in_image(in_coordY - 3, in_coordX - 1) - (m_relDiffTresh * nucleusInt)) > nucleusInt) score++;
    }
    
    return score;
}

} //namespace FeatureHandling
} //namespace VOCPP

