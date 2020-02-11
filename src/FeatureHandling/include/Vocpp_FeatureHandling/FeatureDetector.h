/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/


#ifndef VOCPP_FEATURE_DETECTOR_H
#define VOCPP_FEATURE_DETECTOR_H

#include<Vocpp_Interface/Frame.h>

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief Generic feature detector class. All feature detector implementations have to
  * derive from this class
  */
class FeatureDetector
{
public:
    /**
      * /brief Extract features from a provided image frame. Implementations might place certain
      * restrictions to the image (like being grayscale).
      *
      * \param[in] in_frame image frame from which features shall be extracted
      * \param[out] out_keypoints keypoints extracted from the frame
      * \return True if feature detection successfull, false otherwise
      */
    virtual bool ExtractKeypoints(const Frame& in_frame, std::vector<cv::KeyPoint>& out_keypoints) = 0;
};

/**
  * /brief Instantiate a feature detector given a detector name.
  *
  * \param[in] in_detectorName name of detector
  * \returns pointer to feature detector object if successful, NULL otherwise. Caller takes
  * ownership of object
  */
FeatureDetector* InstantiateFeatureDetector(std::string in_detectorName);

} //namespace FeatureHandling
} //namespace VOCPP


#endif /* VOCPP_FEATURE_DETECTOR_H */
