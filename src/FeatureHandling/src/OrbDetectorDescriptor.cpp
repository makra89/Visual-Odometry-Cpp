/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/OrbDetectorDescriptor.h>
#include<opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace VOCPP
{
namespace FeatureHandling
{


OrbDetectorDescriptor::OrbDetectorDescriptor(const int& in_numOctaves, const float& in_octaveScaleFactor) :
    m_numOctaves(in_numOctaves),
    m_octaveScaleFactor(in_octaveScaleFactor),
    m_fastDetector(),
    m_descriptor(),
    m_octaves()
{
    // Calculate weighted norm (all scales summed up)
    float scaleNorm = 0.0;
    for (int octave = 0; octave < m_numOctaves; octave++)
    {
        scaleNorm += std::pow(m_octaveScaleFactor, octave);
    }

    // Fill octave vector
    for (int octave = 0; octave < m_numOctaves; octave++)
    {
        const float scale = std::pow(m_octaveScaleFactor, octave);
        m_octaves.push_back(Octave{ scale, scale / scaleNorm });
    }
}

bool OrbDetectorDescriptor::ExtractFeatureDescriptions(const Frame& in_frame, const int& in_maxNumFeatures, 
    std::vector<BinaryFeatureDescription>& out_descriptions)
{
    bool ret = true;

    if (in_frame.GetImage().dims != 2)
    {
        ret = false;
        std::cout << "[OrientedFastDetector]: Non-grayscale image has been provided" << std::endl;
    }
    else
    {
        int featureId = 0;
        for (auto octave : m_octaves)
        {
            cv::Mat1f octaveImage;
            if (octave.scale < 1.0)
            {
                cv::resize(in_frame.GetImage(), octaveImage, cv::Size(0, 0), octave.scale, octave.scale);
            }
            else
            {
                octaveImage = in_frame.GetImage();
            }

            
            Frame octaveFrame(octaveImage, in_frame.GetId());
            std::vector<Feature> octaveFeatures;
            //cv::TickMeter tick;
           // tick.start();
            m_fastDetector.ExtractFeatures(octaveFrame, static_cast<int>(in_maxNumFeatures * octave.featureRatio), octaveFeatures);
            //tick.stop();
            //std::cout << "[ORB] OFast " << tick.getTimeMilli() << std::endl;
            //cv::TickMeter tick2;
            //tick2.start();
            std::vector<BinaryFeatureDescription> octaveDescriptions;
            m_descriptor.ComputeDescriptions(octaveFrame, octaveFeatures, octaveDescriptions);
           // tick2.stop();
           // std::cout << "[ORB] rBrief " << tick2.getTimeMilli() << std::endl;
            for (auto desc : octaveDescriptions)
            {
                Feature outFeature{ featureId, desc.GetFeature().frameId,
                    1.F / octave.scale * desc.GetFeature().imageCoordX, 1.F / octave.scale * desc.GetFeature().imageCoordY,
                    desc.GetFeature().response, desc.GetFeature().angle, desc.GetFeature().size / octave.scale, octave.scale };

                out_descriptions.push_back(BinaryFeatureDescription(outFeature, desc.GetDescription()));
                featureId++;
            }
        }
    }


    return ret;
}

} //namespace FeatureHandling
} //namespace VOCPP

