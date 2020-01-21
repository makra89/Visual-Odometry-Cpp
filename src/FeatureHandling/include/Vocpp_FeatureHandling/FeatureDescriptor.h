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
  * /brief Generic feature descriptor class. All feature descriptor implementations have to
  * derive from this class
  */
class FeatureDescriptor
{
public:
    /**
      * /brief Compute descriptions for provided frame
      *
      * \param[in,out] inout_frame frame for which to compute the descriptions, features must be present!
      * \return True if feature description for at least one feature successfull, false otherwise
      */
    virtual bool ComputeDescriptions(Utils::Frame& inout_frame) = 0;
};

/**
  * /brief Instantiate a feature descriptor given a descriptor name.
  *
  * \param[in] in_descriptorName name of descriptor
  * \returns pointer to feature descriptor object if successful, NULL otherwise. Caller takes
  * ownership of object
  */
FeatureDescriptor* InstantiateFeatureDescriptor(std::string in_descriptorName);


} //namespace FeatureHandling
} //namespace VOCPP
