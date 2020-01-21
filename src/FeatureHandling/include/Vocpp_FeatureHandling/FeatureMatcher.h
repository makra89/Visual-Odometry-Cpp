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
  * /brief Generic feature matcher class. All feature matcher implementations have to
  * derive from this class
  */
class FeatureMatcher
{
public:
    /**
      * /brief Match feature descriptions in provided frames and return matches. Features and descriptions have to be present!
      * Features in first frame will be searched in second frame. 
      *
      * \param[in, out] inout_frame1 first frame to match
      * \param[in] inout_frame2 second frame to match
      * \param[in] in_appendMatches specifies whether matches shall be appended to match vectors
      * \return True if feature matching successfull, false otherwise
      */
    virtual bool matchDesriptions(Utils::Frame& inout_frame1, const Utils::Frame& in_frame2, const bool in_appendMatches=false) = 0;
};

/**
  * /brief Instantiate a feature matcher given a matcher name.
  *
  * \param[in] in_matcherName name of feature matcher
  * \returns pointer to feature matcher object if successful, NULL otherwise. Caller takes
  * ownership of object
  */
FeatureMatcher* InstantiateFeatureMatcher(std::string in_matcherName);


} //namespace FeatureHandling
} //namespace VOCPP
