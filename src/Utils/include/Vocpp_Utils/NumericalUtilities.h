/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include <Vocpp_Interface/Types.h>

namespace VOCPP
{
namespace Utils
{

/**
* /brief Simply PI :)
*/
static double PI = 3.141592653589793238463;

/**
* /brief Draw a random int32_t in the range [in_lowerEdge, in_upperEdge]
* if in_fixedSeed is false, the random number generator will be initialized with a random seed
*/
int32_t DrawIntInRange(const int32_t in_lowerEdge, const int32_t in_upperEdge, const bool in_fixedSeed = true);

/**
* /brief Draw a random double in the range [in_lowerEdge, in_upperEdge]
* if in_fixedSeed is false, the random number generator will be initialized with a random seed
*/
double DrawDoubleInRange(const double in_lowerEdge, const double in_upperEdge, const bool in_fixedSeed = true);


} //namespace Utils
} //namespace VOCPP

