/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/NumericalUtilities.h>
#include <gtest/gtest.h>

using VOCPP::Utils::DrawIntInRange;
using VOCPP::Utils::DrawFloatInRange;

// Check that DrawIntInRange provides int in the expected range
TEST(DrawIntInRange, RangeCheck)
{
    int lowerEdge = -10;
    int upperEdge = 10;

    for (uint it = 0U; it < 10000U; it++)
    {
        int rand = DrawIntInRange(lowerEdge, upperEdge);

        EXPECT_GE(rand, lowerEdge);
        EXPECT_LE(rand, upperEdge);
    }
}

// Check that DrawIntInRange provides int with the expect uniform probability distribution
TEST(DrawIntInRange, ProbabilityCheck)
{
    int lowerEdge = -10;
    int upperEdge = 10;

    int elCount[21] = { 0 };

    for (uint it = 0U; it < 100000U; it++)
    {
        int rand = DrawIntInRange(lowerEdge, upperEdge);

        int elIndex = 10 + rand;
        ASSERT_TRUE(elIndex >= 0);
        ASSERT_TRUE(elIndex < 21);

        elCount[elIndex]++;
    }

    // Check element counts, expected is 100k / 21 = 4761
    for (uint it = 0U; it < 21U; it++)
    {
        EXPECT_NEAR(elCount[it], 4761, 300);
    }
}

// Check that DrawFloatInRange provides float in the expected range
TEST(DrawFloatInRange, RangeCheck)
{
    float lowerEdge = -10.0;
    float upperEdge = 10.0;

    for (uint it = 0U; it < 10000U; it++)
    {
        float rand = DrawFloatInRange(lowerEdge, upperEdge);

        EXPECT_GE(rand, lowerEdge);
        EXPECT_LE(rand, upperEdge);
    }
}
