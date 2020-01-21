/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include<Vocpp_Utils/Frame.h>

#include<Vocpp_FeatureHandling/FeatureDetector.h>
#include<Vocpp_FeatureHandling/FeatureDescriptor.h>
#include<Vocpp_FeatureHandling/FeatureMatcher.h>

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{
namespace Master
{
   
/**
  * /brief Master class for calculation of delta poses
  *
  * Until now it is only possible to match features and draw those matches
  */
class Master
{

public:
    
    /**
      * /brief Constructor
      */
    Master();
    
    /**
      * /brief Provide next image frame to master
      */
    bool FeedNextFrame(Utils::Frame& in_frame);

private:

    FeatureHandling::FeatureDetector* m_detector; ///< feature detector
    FeatureHandling::FeatureDescriptor* m_descriptor; ///< feature descriptor
    FeatureHandling::FeatureMatcher* m_matcher; ///< feature matcher

    Utils::Frame m_lastFrame;  ///< last processed frame (fed via Master::FeedNextFrame(Utils::Frame& in_frame)

};

} //namespace Master
} //namespace VOCPP

