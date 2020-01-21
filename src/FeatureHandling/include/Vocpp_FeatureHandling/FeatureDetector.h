/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include<Vocpp_Utils/Frame.h>
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
      * restrictions to the image (like being grayscale). Keypoints will be added to the provided image frame
      *
      * \param[in, out] inout_frame image frame from which features shall be extracted
      * \return True if feature detection successfull, false otherwise
      */
    virtual bool ExtractKeypoints(Utils::Frame& inout_frame) = 0;
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
