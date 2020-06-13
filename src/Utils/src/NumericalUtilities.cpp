/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/NumericalUtilities.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include<opencv2/imgproc.hpp>
#include<iostream>
#include<random>

namespace
{
    const unsigned int seed = 12U;
}

namespace VOCPP
{
namespace Utils
{
    
int DrawIntInRange(const int in_lowerEdge, const int in_upperEdge, const bool in_fixedSeed)
{
    static bool firstDrawn = true;
    if (firstDrawn && !in_fixedSeed)
    {
        srand(static_cast<uint>(time(NULL))); //seed
        firstDrawn = false;
    }
    else if (firstDrawn && in_fixedSeed)
    {
        srand(seed); //seed
        firstDrawn = false;
    }

    return in_lowerEdge + rand() % ((in_upperEdge + 1) - in_lowerEdge);
}

float DrawFloatInRange(const float in_lowerEdge, const float in_upperEdge, const bool in_fixedSeed)
{
    static bool firstDrawn = true;
    if (firstDrawn && !in_fixedSeed)
    {
        srand(static_cast<uint>(time(NULL))); //seed
        firstDrawn = false;
    }
    else if(firstDrawn && in_fixedSeed)
    {
        srand(seed); //seed
        firstDrawn = false;
    }

    return in_lowerEdge + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)* (in_upperEdge - in_lowerEdge);
}

} //namespace Utils
} //namespace VOCPP
