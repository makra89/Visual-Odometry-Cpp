/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/



#include<Vocpp_FeatureHandling/FeatureDescriptor.h>
#include<BriefDescriptor.h>
#include<iostream>

namespace VOCPP
{
namespace FeatureHandling
{

FeatureDescriptor* InstantiateFeatureDescriptor(std::string in_descriptorName)
{
    FeatureDescriptor* ret = NULL;

    if (in_descriptorName == "Brief")
    {
        ret = BriefDescriptor::CreateInstance();
    }
    else
    {
        std::cout << "Unknown feature detector chosen" << std::endl;
    }

    return ret;
}

} //namespace FeatureHandling
} //namespace VOCPP


