/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/OrbDetectorDescriptor.h>
#include <Vocpp_Utils/TracingImpl.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace VOCPP
{
namespace FeatureHandling
{


OrbDetectorDescriptor::OrbDetectorDescriptor(const unsigned int& in_numPyramidLayers, const double& in_layerScaleFactor) :
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
        VOCPP_TRACE_ERROR("[OrientedFastDetector]: Non-grayscale image has been provided");
    }
    else
    {
        // Calculate weighted norm (all scales summed up)
        double scaleNorm = 0.0F;
        for (unsigned int layer = 0U; layer < m_numLayers; layer++)
        {
            scaleNorm += static_cast<double>(std::pow(m_layerScaleFactor, layer));
        }

        // Calculate number of features per layer
        std::vector<int> numFeaturesPerLevel(m_numLayers);
        int numRequestedFeatures = 0;
        for (unsigned int layer = 0U; layer < m_numLayers; layer++)
        {
            const double scale = static_cast<double>(std::pow(m_layerScaleFactor, layer));
            const unsigned int numFeaturesForScale = static_cast<unsigned int>(ceil(scale / scaleNorm * in_maxNumFeatures));
            
            numFeaturesPerLevel[layer] = numFeaturesForScale;
            numRequestedFeatures += numFeaturesForScale;
        }

        // If we have more events than requested, remove from lower layers
        int currentLayer = m_numLayers - 1;
        while (numRequestedFeatures > in_maxNumFeatures)
        {
            if (numFeaturesPerLevel[currentLayer] > (numRequestedFeatures - in_maxNumFeatures))
            {
                numFeaturesPerLevel[currentLayer] -= (numRequestedFeatures - in_maxNumFeatures);
                numRequestedFeatures = in_maxNumFeatures;
            }
            else
            {
                numRequestedFeatures -= numFeaturesPerLevel[currentLayer];
                numFeaturesPerLevel[currentLayer] = 0U;
            }

            currentLayer -= 1;
        }
        
        std::vector<PyramidLayer> pyramid;
        // First layer
        pyramid.push_back(PyramidLayer{ 1.0F, numFeaturesPerLevel[0], in_frame.GetImage()});

        // Fill pyramid
        for (unsigned int layer = 1U; layer < m_numLayers; layer++)
        {
            const double layerScale = static_cast<double>(std::pow(m_layerScaleFactor, layer));
            cv::Size sz(static_cast<unsigned int>(std::round(layerScale * static_cast<double>(pyramid[0].image.cols))), 
                static_cast<unsigned int>(std::round(layerScale * static_cast<double>(pyramid[0].image.rows))));

            cv::Mat1d layerImage;
            cv::resize(pyramid[layer - 1].image, layerImage, sz, 0., 0., cv::INTER_LINEAR_EXACT);
            pyramid.push_back(PyramidLayer{layerScale, numFeaturesPerLevel[layer], layerImage });
        }
        
        
        unsigned int featureId = 0;
        for (auto layer : pyramid)
        {
            if (layer.numFeatures > 0)
            {
                Frame layerFrame(layer.image, in_frame.GetId());
                
                // Get features for layer
                std::vector<Feature> layerFeatures;
                m_fastDetector.ExtractFeatures(layerFrame, layer.numFeatures, layerFeatures);                
                
                // Smooth image using gaussian blur
                cv::Size kernelSize(7U, 7U);       
                cv::GaussianBlur(layer.image, layer.image, kernelSize, 2.F, 2.F, cv::BORDER_REFLECT101);
                std::vector<BinaryFeatureDescription> layerDescriptions;
                m_descriptor.ComputeDescriptions(layerFrame, layerFeatures, layerDescriptions);
                
                for (auto desc : layerDescriptions)
                {
                    Feature feature{ featureId, in_frame.GetId(),
                        1.F / layer.scale * desc.GetFeature().imageCoordX, 1.F / layer.scale * desc.GetFeature().imageCoordY,
                        desc.GetFeature().response, desc.GetFeature().angle, desc.GetFeature().size / layer.scale, layer.scale };
                    out_descriptions.push_back(BinaryFeatureDescription(feature, desc.GetDescription()));
                    featureId++;
                }
            }
        }
    }

    return ret;
}

} //namespace FeatureHandling
} //namespace VOCPP

