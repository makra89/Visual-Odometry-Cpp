/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{
namespace Utils
{

/**
* /brief Simply PI :)
*/
static float PI = 3.141592653589793238463F;

/**
* /brief Draw a random int in the range [in_lowerEdge, in_upperEdge]
* if in_fixedSeed is false, the random number generator will be initialized with a random seed
*/
int DrawIntInRange(const int in_lowerEdge, const int in_upperEdge, const bool in_fixedSeed = true);

/**
* /brief Draw a random float in the range [in_lowerEdge, in_upperEdge]
* if in_fixedSeed is false, the random number generator will be initialized with a random seed
*/
float DrawFloatInRange(const float in_lowerEdge, const float in_upperEdge, const bool in_fixedSeed = true);


} //namespace Utils
} //namespace VOCPP

