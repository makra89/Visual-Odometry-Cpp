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

class OrbDetectorDescriptor
{
public:

    /**
      * /brief Constructor
      */
    OrbDetectorDescriptor(const int& in_numOctaves=7, const float& in_octaveScaleFactor=0.8);

    /**
      * /brief Extract features from a provided grayscale image frame.
      *
      * \param[in] in_frame image frame from which features shall be extracted
      * \param[in] in_maxNumFeatures maximum number of features returned
      * \param[out] out_features features extracted from the frame
      * \return True if at least one feature has been detected, false otherwise
      */
    bool ExtractFeatureDescriptions(const Frame& in_frame, const int& in_maxNumFeatures, std::vector<BinaryFeatureDescription>& out_descriptions);

private:
    // It is not allowed to copy the detector directly
    OrbDetectorDescriptor& operator=(const OrbDetectorDescriptor&);
    OrbDetectorDescriptor(const OrbDetectorDescriptor&);

    struct Octave
    {
        float scale;
        float featureRatio;
    };

    const int m_numOctaves;
    const float m_octaveScaleFactor;

    FeatureHandling::OrientedFastDetector m_fastDetector;
    FeatureHandling::BriefDescriptor m_descriptor;
    std::vector<Octave> m_octaves;
};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_ORB_DETECTOR_DESCRIPTOR_H */
