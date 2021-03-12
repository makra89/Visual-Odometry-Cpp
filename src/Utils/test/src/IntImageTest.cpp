/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/IntImage.h>
#include <gtest/gtest.h>

// Check content im integral image
TEST(IntImageTest, GetIntensityTest)
{
    cv::Mat1d allOnes = cv::Mat1d::ones(3, 4);

    VOCPP::Utils::IntImage intImage(allOnes);

    // First column
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(0, 0), 1);
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(1, 0), 2);
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(2, 0), 3);

    // Second column
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(0, 1), 2);
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(1, 1), 4);
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(2, 1), 6);

    // Third column
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(0, 2), 3);
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(1, 2), 6);
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(2, 2), 9);

    // Fourth column
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(0, 3), 4);
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(1, 3), 8);
    EXPECT_DOUBLE_EQ(intImage.GetIntensity(2, 3), 12);
}

TEST(IntImageTest, GetAreaAroundPixelTest)
{
    cv::Mat1d allOnes = cv::Mat1d::ones(14, 15);

    VOCPP::Utils::IntImage intImage(allOnes);

    // Check area in a patch with radius 4 around the center pixel
    // --> 9x9 patch --> should be 81
    double area = 0;
    EXPECT_TRUE(intImage.GetAreaAroundPixel(5, 6, 4, area));
    EXPECT_DOUBLE_EQ(area, 81);

    // Check area in a 5x5 patch --> should be 25
    EXPECT_TRUE(intImage.GetAreaAroundPixel(5, 6, 2, area));
    EXPECT_DOUBLE_EQ(area, 25);

    // Check area in a 3x3 patch --> should be 9
    EXPECT_TRUE(intImage.GetAreaAroundPixel(5, 6, 1, area));
    EXPECT_DOUBLE_EQ(area, 9);
}

