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


OrbDetectorDescriptor::OrbDetectorDescriptor(const unsigned int& in_numPyramidLayers, const float& in_layerScaleFactor) :
    m_numLayers(in_numPyramidLayers),
    m_layerScaleFactor(in_layerScaleFactor),
    m_fastDetector(),
    m_descriptor()
{
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
        // Calculate weighted norm (all scales summed up)
        float scaleNorm = 0.0F;
        for (unsigned int layer = 0U; layer < m_numLayers; layer++)
        {
            const float scaleSq = static_cast<float>(std::pow(std::pow(m_layerScaleFactor, layer), 2));
            scaleNorm += scaleSq;
        }
        
        std::vector<PyramidLayer> pyramid;
        // First layer
        pyramid.push_back(PyramidLayer{ 1.0F, 1.0F / scaleNorm, in_frame.GetImage()});

        // Fill pyramid
        for (unsigned int layer = 1U; layer < m_numLayers; layer++)
        {
            const float scale = static_cast<float>(std::pow(m_layerScaleFactor, layer));
            const float scaleSq = static_cast<float>(std::pow(std::pow(m_layerScaleFactor, layer), 2));

            // Kernel size for smoothing
            cv::Size kernelSize(5U, 5U);

            cv::Mat1f layerImage;
            cv::GaussianBlur(pyramid[layer - 1].image, layerImage, kernelSize, static_cast<float>(kernelSize.width), static_cast<float>(kernelSize.height));
            cv::resize(layerImage, layerImage, cv::Size(0, 0), scale, scale);

            pyramid.push_back(PyramidLayer{ scale, scaleSq / scaleNorm, layerImage });
        }
        
        
        unsigned int featureId = 0;
        for (auto layer : pyramid)
        { 
            Frame layerFrame(layer.image, in_frame.GetId());
            std::vector<Feature> layerFeatures;
            m_fastDetector.ExtractFeatures(layerFrame, static_cast<int>(in_maxNumFeatures * layer.featureRatio), layerFeatures);
            std::vector<BinaryFeatureDescription> layerDescriptions;
            m_descriptor.ComputeDescriptions(layerFrame, layerFeatures, layerDescriptions);

            for (auto desc : layerDescriptions)
            {
                Feature outFeature{ featureId, desc.GetFeature().frameId,
                    1.F / layer.scale * desc.GetFeature().imageCoordX, 1.F / layer.scale * desc.GetFeature().imageCoordY,
                    desc.GetFeature().response, desc.GetFeature().angle, desc.GetFeature().size / layer.scale, layer.scale };

                out_descriptions.push_back(BinaryFeatureDescription(outFeature, desc.GetDescription()));
                featureId++;
            }
        }
    }


    return ret;
}

} //namespace FeatureHandling
} //namespace VOCPP

