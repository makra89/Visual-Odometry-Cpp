/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <gtest/gtest.h>

using namespace VOCPP::Utils;

TEST(ExtractLocalMaximaTest, BlockMatrix)
{
    // Construct frame filled with ones
    cv::Mat ones = GetWindowKernel(5);

    std::vector<cv::Point2f> max;
    ExtractLocalMaxima(ones, 2U, max, 0U);
    
    // We expect 5 * 5 maxima since all values are equal
    EXPECT_EQ(max.size(), 25);
}

TEST(ExtractLocalMaximaTest, OneMax)
{
    // Construct image with one center maxima
    cv::Mat ones = cv::Mat::zeros(5, 5, CV_32F);
    ones.at<float>(2, 2) = 1.0;

    std::vector<cv::Point2f> max;
    ExtractLocalMaxima(ones, 2U, max, 0U);

    // We expect one single maximum
    EXPECT_EQ(max.size(), 1);
    EXPECT_FLOAT_EQ(max[0].x, 2.0);
    EXPECT_FLOAT_EQ(max[0].y, 2.0);
}

TEST(ExtractLocalMaximaTest, TestDistanceCheck)
{
    // Construct image with three close-by maxima
    cv::Mat ones = cv::Mat::zeros(5, 5, CV_32F);
    ones.at<float>(2, 2) = 1.0;
    ones.at<float>(2, 3) = 3.0;
    ones.at<float>(3, 2) = 4.0;

    std::vector<cv::Point2f> max;
    ExtractLocalMaxima(ones, 0U, max, 0U);

    // Without any distance check we expect three maxima
    EXPECT_EQ(max.size(), 3);
    EXPECT_FLOAT_EQ(max[0].x, 2.0);
    EXPECT_FLOAT_EQ(max[0].y, 2.0);
    EXPECT_FLOAT_EQ(max[1].x, 2.0);
    EXPECT_FLOAT_EQ(max[1].y, 3.0);
    EXPECT_FLOAT_EQ(max[2].x, 3.0);
    EXPECT_FLOAT_EQ(max[2].y, 2.0);

    max.clear();
    // Do it again, this time with a minimum distance
    ExtractLocalMaxima(ones, 1U, max, 0U);

    // Only one maximum survives
    EXPECT_EQ(max.size(), 1);
    EXPECT_FLOAT_EQ(max[0].x, 2.0);
    EXPECT_FLOAT_EQ(max[0].y, 3.0);
}

TEST(ExtractLocalMaximaTest, SubPixPrecisionTest)
{
    // Construct image with two maxima
    cv::Mat ones = cv::Mat::zeros(5, 5, CV_32F);
    ones.at<float>(2, 2) = 2.0;
    ones.at<float>(2, 3) = 1.0;

    std::vector<cv::Point2f> max;
    //Average of a pixel distance of 1
    ExtractLocalMaxima(ones, 1U, max, 1U);

    // Test averaged pixel position
    EXPECT_EQ(max.size(), 1);
    EXPECT_NEAR(max[0].x, 2.33, 1e-2);
    EXPECT_FLOAT_EQ(max[0].y, 2.0);

    max.clear();
    // Now test what happens when the averaging distance
    // exceeds the image range, we expect to have one maximum
    ExtractLocalMaxima(ones, 1U, max, 3U);
    EXPECT_EQ(max.size(), 1);
    EXPECT_FLOAT_EQ(max[0].y, 2.0);
    EXPECT_FLOAT_EQ(max[0].y, 2.0);
}