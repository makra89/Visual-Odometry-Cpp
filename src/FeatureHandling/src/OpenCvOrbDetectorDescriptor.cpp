/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/OpenCvOrbDetectorDescriptor.h>
#include <opencv2/opencv.hpp>

namespace VOCPP
{
namespace FeatureHandling
{


OpenCvOrbDetectorDescriptor::OpenCvOrbDetectorDescriptor(const unsigned int& in_numPyramidLayers, const float& in_layerScaleFactor)
{
    m_orb = cv::ORB::create();
    m_orb->setNLevels(in_numPyramidLayers);
    m_orb->setScaleFactor(1.F/in_layerScaleFactor);
}

bool OpenCvOrbDetectorDescriptor::ExtractFeatureDescriptions(const Frame& in_frame, const int& in_maxNumFeatures, 
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
        m_orb->setMaxFeatures(in_maxNumFeatures);
        cv::Mat image;
        in_frame.GetImage().convertTo(image, CV_8UC1, 255.0);
        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        m_orb->detect(image, keypoints);
        m_orb->compute(image, keypoints, descriptors);

        unsigned int featureId = 0U;
        for (int keyPointId = 0; keyPointId < keypoints.size(); keyPointId++)
        {
            Feature feat{ featureId, in_frame.GetId(), keypoints[keyPointId].pt.x, keypoints[keyPointId].pt.y, keypoints[keyPointId].response, keypoints[keyPointId].octave };
            std::vector<uint8_t> description;
            for (int blockId = 0; blockId < 32; blockId++)
            {
                description.push_back(descriptors.at<uint8_t>(keyPointId, blockId));
            }

            out_descriptions.push_back(BinaryFeatureDescription(feat, description));

            featureId++;


        }
    }

    return ret;
}

} //namespace FeatureHandling
} //namespace VOCPP

