/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include<input.h>

#include<Vocpp_FeatureHandling/OrFastDetector.h>

// Check that all three edges of a triangle are detected
// Check that both intensity check scenarios are covered:
//  12 pixels are significantly lower in intensity than the central one
//  12 pixels are significantly higher in intensity than the central one
TEST(OrFastDetectorTest, DetectTriangleEdges)
{
    cv::Mat grayScaleImg;
    cv::cvtColor(cv::imread(testDirectory + "Triangle.jpg", 1), grayScaleImg, cv::COLOR_BGR2GRAY);
    grayScaleImg.convertTo(grayScaleImg, CV_32F, 1.0 / 255.0);

    VOCPP::FeatureHandling::OrientedFastDetector detector(0.2F, 12, 3, 5);

    VOCPP::Frame frameFirst(grayScaleImg.ptr<float>(0), grayScaleImg.cols, grayScaleImg.rows, 1);
    std::vector<VOCPP::FeatureHandling::Feature> features;
    // The three edges should be the strongest features
    EXPECT_TRUE(detector.ExtractFeatures(frameFirst, 3, features));
    ASSERT_TRUE(features.size() == 3);
    
    // Features are unordered since they are sorted with respect to response
    // Feature one (upper edge)
    EXPECT_EQ(features[0].imageCoordX, 223);
    EXPECT_EQ(features[0].imageCoordY, 120);
    EXPECT_NEAR(features[0].angle * 180 / 3.141, -90, 10);

    // Feature two (lower left edge)
    EXPECT_EQ(features[1].imageCoordX, 18);
    EXPECT_EQ(features[1].imageCoordY, 483);
    EXPECT_NEAR(features[1].angle * 180 / 3.141, 150, 10);

    // Feature three (lower right edge)
    EXPECT_EQ(features[2].imageCoordX, 419);
    EXPECT_EQ(features[2].imageCoordY, 482);
    EXPECT_NEAR(features[2].angle * 180 / 3.141, 36, 10);

    // Check response sorting
    EXPECT_GT(features[0].response, features[1].response);
    EXPECT_GT(features[1].response, features[2].response);
    
    // Also check what happens when we negate the image, this checks the case 
    // "surrounding pixels are lower in intensity"
    grayScaleImg.convertTo(grayScaleImg, CV_8U, 255.0);
    cv::bitwise_not(grayScaleImg, grayScaleImg);
    grayScaleImg.convertTo(grayScaleImg, CV_32F, 1.0 / 255.0);
    VOCPP::Frame frameSecond(grayScaleImg.ptr<float>(0), grayScaleImg.cols, grayScaleImg.rows, 1);
    features.clear();
    EXPECT_TRUE(detector.ExtractFeatures(frameSecond, 3, features));

    ASSERT_TRUE(features.size() == 3);

    // Features are unordered since they are sorted with respect to response
    // Feature one (upper edge)
    EXPECT_EQ(features[0].imageCoordX, 223);
    EXPECT_EQ(features[0].imageCoordY, 120);
    EXPECT_NEAR(features[0].angle * 180 / 3.141, 90, 10);

    // Feature two (lower right edge)
    EXPECT_EQ(features[1].imageCoordX, 18);
    EXPECT_EQ(features[1].imageCoordY, 483);
    EXPECT_NEAR(features[1].angle * 180 / 3.141, -31, 10);

    // Feature three (lower left edge)
    EXPECT_EQ(features[2].imageCoordX, 419);
    EXPECT_EQ(features[2].imageCoordY, 482);

    // Check response sorting
    EXPECT_GT(features[0].response, features[1].response);
    EXPECT_GT(features[1].response, features[2].response);
}
