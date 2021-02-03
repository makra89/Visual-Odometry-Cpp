/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_OPENCV_ORB_DETECTOR_DESCRIPTOR_H
#define VOCPP_OPENCV_ORB_DETECTOR_DESCRIPTOR_H

#include <Vocpp_Interface/Frame.h>
#include <Vocpp_FeatureHandling/Common.h>
#include <opencv2/features2d.hpp>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief OpenCv implementation of OBR feature detector and descriptor 
  */
class OpenCvOrbDetectorDescriptor
{
public:

    /**
      * /brief Constructor
      */
    OpenCvOrbDetectorDescriptor(const unsigned int& in_numPyramidLayers=1U, const float& in_layerScaleFactor=0.7);

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
    OpenCvOrbDetectorDescriptor& operator=(const OpenCvOrbDetectorDescriptor&);
    OpenCvOrbDetectorDescriptor(const OpenCvOrbDetectorDescriptor&);

    cv::Ptr<cv::ORB> m_orb;
};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_OPENCV_ORB_DETECTOR_DESCRIPTOR_H */
