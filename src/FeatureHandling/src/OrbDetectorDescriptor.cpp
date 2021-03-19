/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/OrbDetectorDescriptor.h>
#include <Vocpp_Utils/TracingImpl.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace
{

int32_t roundDouble(const double& in_num)
{
    return static_cast<int32_t>(in_num + 0.5);
}

}
namespace VOCPP
{
namespace FeatureHandling
{


OrbDetectorDescriptor::OrbDetectorDescriptor(const uint32_t& in_numPyramidLayers, const double& in_layerScaleFactor) :
    m_numLayers(in_numPyramidLayers),
    m_layerScaleFactor(in_layerScaleFactor),
    m_fastDetector(),
    m_descriptor()
{
}

bool OrbDetectorDescriptor::ExtractFeatureDescriptions(const Frame& in_frame, const uint32_t& in_maxNumFeatures, 
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
        for (uint32_t layer = 0U; layer < m_numLayers; layer++)
        {
            scaleNorm += static_cast<double>(std::pow(m_layerScaleFactor, static_cast<int32_t>(layer)));
        }

        // Calculate number of features per layer
        std::vector<uint32_t> numFeaturesPerLevel(m_numLayers);
        uint32_t numRequestedFeatures = 0U;
        for (uint32_t layer = 0U; layer < m_numLayers; layer++)
        {
            const double scale = static_cast<double>(std::pow(m_layerScaleFactor, static_cast<int32_t>(layer)));
            const uint32_t numFeaturesForScale = static_cast<uint32_t>(ceil(scale / scaleNorm * in_maxNumFeatures));
            
            numFeaturesPerLevel[layer] = numFeaturesForScale;
            numRequestedFeatures += numFeaturesForScale;
        }

        // If we have more events than requested, remove from lower layers
        int32_t currentLayer = m_numLayers - 1;
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
        PyramidLayer firstLayer = { 1.0F, numFeaturesPerLevel[0], in_frame.GetImage() };
        pyramid.push_back(firstLayer);

        // Fill pyramid
        for (uint32_t layerIdx = 1U; layerIdx < m_numLayers; layerIdx++)
        {
            const double layerScale = static_cast<double>(std::pow(m_layerScaleFactor, static_cast<int32_t>(layerIdx)));
            cv::Size sz(static_cast<uint32_t>(roundDouble(layerScale * static_cast<double>(pyramid[0].image.cols))),
                static_cast<uint32_t>(roundDouble(layerScale * static_cast<double>(pyramid[0].image.rows))));

            cv::Mat1d layerImage;
            cv::resize(pyramid[layerIdx - 1].image, layerImage, sz, 0., 0., cv::INTER_LINEAR);
            PyramidLayer layer = { layerScale, numFeaturesPerLevel[layerIdx], layerImage };
            pyramid.push_back(layer);
        }
        
        
        uint32_t featureId = 0;
        
        for (uint32_t layerIdx = 0U; layerIdx < pyramid.size(); layerIdx++)
        {
            if (pyramid[layerIdx].numFeatures > 0)
            {
                Frame layerFrame(pyramid[layerIdx].image, in_frame.GetId());
                
                // Get features for layer
                std::vector<Feature> layerFeatures;
                m_fastDetector.ExtractFeatures(layerFrame, pyramid[layerIdx].numFeatures, layerFeatures);
                
                // Smooth image using gaussian blur
                cv::Size kernelSize(7U, 7U);       
                cv::GaussianBlur(pyramid[layerIdx].image, pyramid[layerIdx].image, kernelSize, 2.F, 2.F, cv::BORDER_REFLECT101);
                std::vector<BinaryFeatureDescription> layerDescriptions;
                m_descriptor.ComputeDescriptions(layerFrame, layerFeatures, layerDescriptions);
                
                for (uint32_t descIdx = 0U; descIdx < layerDescriptions.size(); descIdx++)
                {
                    Feature feature = { featureId, in_frame.GetId(),
                        1.F / pyramid[layerIdx].scale * layerDescriptions[descIdx].GetFeature().imageCoordX, 1.F / pyramid[layerIdx].scale * layerDescriptions[descIdx].GetFeature().imageCoordY,
                        layerDescriptions[descIdx].GetFeature().response, layerDescriptions[descIdx].GetFeature().angle, layerDescriptions[descIdx].GetFeature().size / pyramid[layerIdx].scale, pyramid[layerIdx].scale };
                    out_descriptions.push_back(BinaryFeatureDescription(feature, layerDescriptions[descIdx].GetDescription()));
                    featureId++;
                }
            }
        }
    }

    return ret && (out_descriptions.size() > 0U);
}

} //namespace FeatureHandling
} //namespace VOCPP

