/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include<Vocpp_FeatureHandling/FeatureDetector.h>
#include<HarrisEdgeDetector.h>
#include<iostream>


namespace VOCPP
{
namespace FeatureHandling
{

FeatureDetector* InstantiateFeatureDetector(std::string detectorName)
{
    FeatureDetector* ret = NULL;

    if (detectorName == "Harris")
    {
        ret = HarrisEdgeDetector::CreateInstance();
    }
    else
    {
        std::cout << "Unknown feature detector chosen" << std::endl;
    }

    return ret;
}

} //namespace FeatureHandling
} //namespace VOCPP


