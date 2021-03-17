/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <gtest/gtest.h>

int32_t main(int32_t argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    int32_t testResult = RUN_ALL_TESTS();

    return testResult;
}
