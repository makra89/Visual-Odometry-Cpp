/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_ORB_DETECTOR_DESCRIPTOR_H
#define VOCPP_ORB_DETECTOR_DESCRIPTOR_H

#include <Vocpp_Interface/Frame.h>
#include <Vocpp_FeatureHandling/Common.h>
#include <Vocpp_FeatureHandling/OrFastDetector.h>
#include <Vocpp_FeatureHandling/BriefDescriptor.h>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief OBR feature detector and descriptor
  * Like described in the ORB paper an oriented FAST detector and 
  * rotated BRIEF descriptor is used.
  */
class OrbDetectorDescriptor
{
public:

    /**
      * /brief Constructor
      */
    OrbDetectorDescriptor(const unsigned int& in_numPyramidLayers=4U, const float& in_layerScaleFactor=0.8);

    /**
      * /brief Extract descriptions from a provided grayscale image frame.
      *
      * \param[in] in_frame image frame from which features shall be extracted
      * \param[in] in_maxNumFeatures maximum number of features returned
      * \param[out] out_descriptions feature descriptions extracted from the frame
      * \return True if at least one description has been detected, false otherwise
      */
    bool ExtractFeatureDescriptions(const Frame& in_frame, const int& in_maxNumFeatures, std::vector<BinaryFeatureDescription>& out_descriptions);

private:
    // It is not allowed to copy the detector directly
    OrbDetectorDescriptor& operator=(const OrbDetectorDescriptor&);
    OrbDetectorDescriptor(const OrbDetectorDescriptor&);

    struct PyramidLayer
    {
        float scale;
        float featureRatio;
        cv::Mat1f image;
    };

    const unsigned int m_numLayers; ///< number of pyramid layers generated for one frame
    const float m_layerScaleFactor; ///< scale factor used to downsample the image for each octave

    FeatureHandling::OrientedFastDetector m_fastDetector;
    FeatureHandling::BriefDescriptor m_descriptor;
};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_ORB_DETECTOR_DESCRIPTOR_H */
