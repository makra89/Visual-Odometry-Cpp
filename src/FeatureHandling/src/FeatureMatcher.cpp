/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include<Vocpp_FeatureHandling/FeatureMatcher.h>
#include<BruteForceMatcher.h>
#include<iostream>


namespace VOCPP
{
namespace FeatureHandling
{

FeatureMatcher* InstantiateFeatureMatcher(std::string in_matcherName)
{
    FeatureMatcher* ret = NULL;

    if (in_matcherName == "BruteForce")
    {
        ret = BruteForceMatcher::CreateInstance();
    }
    else
    {
        std::cout << "Unknown feature detector chosen" << std::endl;
    }

    return ret;
}

} //namespace FeatureHandling
} //namespace VOCPP

