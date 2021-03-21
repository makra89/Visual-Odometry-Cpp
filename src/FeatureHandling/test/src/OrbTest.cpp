/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include<input.h>

#include<Vocpp_FeatureHandling/OrbDetectorDescriptor.h>
#include<Vocpp_FeatureHandling/LshMatcher.h>

// Check that all three edges of a triangle are detected
// Check that both intensity check scenarios are covered:
//  12 pixels are significantly lower in intensity than the central one
//  12 pixels are significantly higher in intensity than the central one
TEST(OrbTest, DetectTriangleEdges_OneLayer)
{
    cv::Mat grayScaleImg;
    cv::cvtColor(cv::imread(testDirectory + "Triangle.jpg", 1), grayScaleImg, cv::COLOR_BGR2GRAY);
    grayScaleImg.convertTo(grayScaleImg, CV_64FC1, 1.0 / 255.0);
    // Copy the image to a larger one to take care of the boundaries
    cv::Mat img = cv::Mat::ones(700, 700, CV_64FC1);
    grayScaleImg.copyTo(img(cv::Rect(75, 75, grayScaleImg.cols, grayScaleImg.rows)));

    VOCPP::FeatureHandling::OrbDetectorDescriptor detector(1U /*one layer*/, 0.7F /*scale factor*/);

    VOCPP::Frame frameFirst(img.ptr<double>(0), img.cols, img.rows, 1);
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptions;
    // The three edges should be the strongest features
    EXPECT_TRUE(detector.ExtractFeatureDescriptions(frameFirst, 3U, descriptions));
    ASSERT_TRUE(descriptions.size() == 3);
    
    // Features are unordered since they are sorted with respect to response
    // Feature one (upper edge)
    EXPECT_EQ(descriptions[0].GetFeature().imageCoordX, 298);
    EXPECT_EQ(descriptions[0].GetFeature().imageCoordY, 195);
    EXPECT_NEAR(descriptions[0].GetFeature().angle * 180 / 3.141, -90, 10);

    // Feature two (lower left edge)
    EXPECT_EQ(descriptions[1].GetFeature().imageCoordX, 93);
    EXPECT_EQ(descriptions[1].GetFeature().imageCoordY, 558);
    EXPECT_NEAR(descriptions[1].GetFeature().angle * 180 / 3.141, 150, 10);

    // Feature three (lower right edge)
    EXPECT_EQ(descriptions[2].GetFeature().imageCoordX, 494);
    EXPECT_EQ(descriptions[2].GetFeature().imageCoordY, 557);
    EXPECT_NEAR(descriptions[2].GetFeature().angle * 180 / 3.141, 36, 10);

    // Check response sorting
    EXPECT_GT(descriptions[0].GetFeature().response, descriptions[1].GetFeature().response);
    EXPECT_GT(descriptions[1].GetFeature().response, descriptions[2].GetFeature().response);
    
    // Also check what happens when we negate the image, this checks the case 
    // "surrounding pixels are lower in intensity"
    grayScaleImg.convertTo(grayScaleImg, CV_8U, 255.0);
    cv::bitwise_not(grayScaleImg, grayScaleImg);
    grayScaleImg.convertTo(grayScaleImg, CV_64F, 1.0 / 255.0);
    img = cv::Mat::zeros(700, 700, CV_64FC1);
    grayScaleImg.copyTo(img(cv::Rect(75, 75, grayScaleImg.cols, grayScaleImg.rows)));
    VOCPP::Frame frameSecond(img.ptr<double>(0), img.cols, img.rows, 1);
    descriptions.clear();
    EXPECT_TRUE(detector.ExtractFeatureDescriptions(frameSecond, 3U, descriptions));
    ASSERT_TRUE(descriptions.size() == 3);

    // Features are unordered since they are sorted with respect to response
    // Feature one (upper edge)
    EXPECT_EQ(descriptions[0].GetFeature().imageCoordX, 298);
    EXPECT_EQ(descriptions[0].GetFeature().imageCoordY, 195);
    EXPECT_NEAR(descriptions[0].GetFeature().angle * 180 / 3.141, 90, 10);

    // Feature two (lower right edge)
    EXPECT_EQ(descriptions[1].GetFeature().imageCoordX, 93);
    EXPECT_EQ(descriptions[1].GetFeature().imageCoordY, 558);
    EXPECT_NEAR(descriptions[1].GetFeature().angle * 180 / 3.141, -31, 10);

    // Feature three (lower left edge)
    EXPECT_EQ(descriptions[2].GetFeature().imageCoordX, 494);
    EXPECT_EQ(descriptions[2].GetFeature().imageCoordY, 557);

    // Check response sorting
    EXPECT_GT(descriptions[0].GetFeature().response, descriptions[1].GetFeature().response);
    EXPECT_GT(descriptions[1].GetFeature().response, descriptions[2].GetFeature().response);
}


// Check that the ORB descriptor + matcher is rotation invariant
TEST(OrbTestWithMatching, RotationInvariance_OneLayer)
{
    cv::Mat grayScaleImg;
    cv::cvtColor(cv::imread(testDirectory + "far-north.jpg", 1), grayScaleImg, cv::COLOR_BGR2GRAY);
    grayScaleImg.convertTo(grayScaleImg, CV_64FC1, 1.0 / 255.0);
    cv::Mat rotatedGrayScaleImg;
    cv::rotate(grayScaleImg, rotatedGrayScaleImg, cv::ROTATE_90_CLOCKWISE);

    VOCPP::FeatureHandling::OrbDetectorDescriptor detector(1U /*one layers*/, 0.7F /*scale factor*/);

    VOCPP::Frame frameUnscaled(grayScaleImg.ptr<double>(0), grayScaleImg.cols, grayScaleImg.rows, 1);
    VOCPP::Frame frameRotated(rotatedGrayScaleImg.ptr<double>(0), rotatedGrayScaleImg.cols, rotatedGrayScaleImg.rows, 1);
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptions;
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptionsRotated;
    EXPECT_TRUE(detector.ExtractFeatureDescriptions(frameUnscaled, 500U, descriptions));
    EXPECT_TRUE(detector.ExtractFeatureDescriptions(frameRotated, 500U, descriptionsRotated));

    // Instantiate Matcher
    VOCPP::FeatureHandling::LshMatcher matcher;
    std::vector<VOCPP::FeatureHandling::BinaryDescriptionMatch> matches;
    matcher.MatchDesriptions(descriptions, descriptionsRotated, matches);
    EXPECT_GE(matches.size(), 485);

    for (uint32_t idx = 0U; idx < matches.size(); idx++)
    {
        EXPECT_EQ(matches[idx].GetFirstFeature().imageCoordX, matches[idx].GetSecondFeature().imageCoordY);
        EXPECT_EQ(matches[idx].GetFirstFeature().imageCoordY, (grayScaleImg.rows - 1U) - matches[idx].GetSecondFeature().imageCoordX);
        
        // All features should have unit scale (only one layer)
        EXPECT_EQ(matches[idx].GetFirstFeature().scale, 1.0);
        EXPECT_EQ(matches[idx].GetSecondFeature().scale, 1.0);

        // We expect that the feature angle differ by pi/2
        double angleDiff = std::abs(matches[idx].GetFirstFeature().angle - matches[idx].GetSecondFeature().angle);
        if (angleDiff > CV_PI) angleDiff -= static_cast<double>(CV_PI);
        EXPECT_NEAR(angleDiff, CV_PI / 2., 0.2);
    }   
}

// Check that the ORB descriptor + matcher is rotation invariant using three layers
TEST(OrbTestWithMatching, RotationInvariance_ThreeLayers)
{
    cv::Mat grayScaleImg;
    cv::cvtColor(cv::imread(testDirectory + "far-north.jpg", 1), grayScaleImg, cv::COLOR_BGR2GRAY);
    grayScaleImg.convertTo(grayScaleImg, CV_64FC1, 1.0 / 255.0);

    cv::Mat rotatedGrayScaleImg;
    cv::rotate(grayScaleImg, rotatedGrayScaleImg, cv::ROTATE_90_CLOCKWISE);

    VOCPP::FeatureHandling::OrbDetectorDescriptor detector(3U /*three layers*/, 0.5F /*scale factor*/);

    VOCPP::Frame frameUnscaled(grayScaleImg.ptr<double>(0), grayScaleImg.cols, grayScaleImg.rows, 1);
    VOCPP::Frame frameRotated(rotatedGrayScaleImg.ptr<double>(0), rotatedGrayScaleImg.cols, rotatedGrayScaleImg.rows, 1);
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptions;
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptionsRotated;
    EXPECT_TRUE(detector.ExtractFeatureDescriptions(frameUnscaled, 500U, descriptions));
    EXPECT_TRUE(detector.ExtractFeatureDescriptions(frameRotated, 500U, descriptionsRotated));

    // Instantiate Matcher
    VOCPP::FeatureHandling::LshMatcher matcher;
    std::vector<VOCPP::FeatureHandling::BinaryDescriptionMatch> matches;
    matcher.MatchDesriptions(descriptions, descriptionsRotated, matches);
    EXPECT_GE(matches.size(), 400);

    cv::resize(grayScaleImg, grayScaleImg, cv::Size(0, 0), 1.0, grayScaleImg.cols/grayScaleImg.rows);

    for (uint32_t idx = 0U; idx < matches.size(); idx++)
    {
        EXPECT_EQ(matches[idx].GetFirstFeature().scale, matches[idx].GetSecondFeature().scale);
        if (matches[idx].GetFirstFeature().scale == 1.0)
        {
            EXPECT_EQ(matches[idx].GetFirstFeature().imageCoordX, matches[idx].GetSecondFeature().imageCoordY);
            EXPECT_EQ(matches[idx].GetFirstFeature().imageCoordY, (grayScaleImg.rows - 1U) - matches[idx].GetSecondFeature().imageCoordX);
        }
        else
        {
            EXPECT_NEAR(matches[idx].GetFirstFeature().imageCoordX, matches[idx].GetSecondFeature().imageCoordY, 0.0);
            // Todo: This is still worrysome!
            EXPECT_NEAR(matches[idx].GetFirstFeature().imageCoordY, (grayScaleImg.rows - 1U) - matches[idx].GetSecondFeature().imageCoordX, 1.0);
        }

        // We expect that the feature angle differ by pi/2
        double angleDiff = std::abs(matches[idx].GetFirstFeature().angle - matches[idx].GetSecondFeature().angle);
        if (angleDiff > CV_PI) angleDiff -= static_cast<double>(CV_PI);
        EXPECT_NEAR(angleDiff, CV_PI / 2., 0.3);
    }
}



// Check that the ORB descriptor + matcher is scale invariant
TEST(OrbTestWithMatching, ScaleInvariance)
{
    cv::Mat grayScaleImg;
    cv::cvtColor(cv::imread(testDirectory + "far-north.jpg", 1), grayScaleImg, cv::COLOR_BGR2GRAY);
    grayScaleImg.convertTo(grayScaleImg, CV_64FC1, 1.0 / 255.0);
    cv::Mat rescaledGrayScaleImg;
    cv::resize(grayScaleImg, rescaledGrayScaleImg, cv::Size(0, 0), 0.5, 0.5);

    VOCPP::FeatureHandling::OrbDetectorDescriptor detector(8U /*eight layers*/, 0.833F /*scale factor*/);

    VOCPP::Frame frameUnscaled(grayScaleImg.ptr<double>(0), grayScaleImg.cols, grayScaleImg.rows, 1);
    VOCPP::Frame frameRescaled(rescaledGrayScaleImg.ptr<double>(0), rescaledGrayScaleImg.cols, rescaledGrayScaleImg.rows, 1);
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptionsUnscaled;
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptionsRescaled;
    EXPECT_TRUE(detector.ExtractFeatureDescriptions(frameUnscaled, 1000U, descriptionsUnscaled));
    EXPECT_TRUE(detector.ExtractFeatureDescriptions(frameRescaled, 1000U, descriptionsRescaled));
    
    // Instantiate Matcher
    VOCPP::FeatureHandling::LshMatcher matcher;
    std::vector<VOCPP::FeatureHandling::BinaryDescriptionMatch> matches;

    matcher.MatchDesriptions(descriptionsUnscaled, descriptionsRescaled, matches);
    EXPECT_GE(matches.size(), 200);

    uint32_t numOutlierAngle = 0U;

    for (uint32_t idx = 0U; idx < matches.size(); idx++)
    {
        // We expect that we have to downscale the unscaled image to find matches
        EXPECT_LT(matches[idx].GetFirstFeature().scale, matches[idx].GetSecondFeature().scale);
        // We expect that the feature angle should be similar
        // TODO: Has been deactivated, there are some outliers
        if (abs(matches[idx].GetFirstFeature().angle - matches[idx].GetSecondFeature().angle) > 0.2)
        {
            numOutlierAngle++;
        }
    }
    // Expect low number of outliers
    EXPECT_LE(numOutlierAngle, 7U);

    /* Comment in for visualization
    cv::Mat matchImg = cv::Mat::ones(1000, 2000, CV_64FC1);
    grayScaleImg.copyTo(matchImg(cv::Rect(0, 0, grayScaleImg.cols, grayScaleImg.rows)));
    rescaledGrayScaleImg.copyTo(matchImg(cv::Rect(1000, 0, rescaledGrayScaleImg.cols, rescaledGrayScaleImg.rows)));
    cv::cvtColor(matchImg, matchImg, cv::COLOR_GRAY2BGR);

    std::vector<cv::Point2d> pRescaled;
    std::vector<cv::Point2d> pUnscaled;
    VOCPP::FeatureHandling::GetMatchingPoints(matches, pUnscaled, pRescaled);
    for (uint32_t idx = 0U; idx < matches.size(); idx++)
    {
        cv::circle(matchImg, cv::Point2d(matches[idx].GetFirstDescription().GetFeature().imageCoordX, matches[idx].GetFirstDescription().GetFeature().imageCoordY), 5, cv::Scalar(0, 0.0, 255.0), 2);
        cv::circle(matchImg, cv::Point2d(1000 + matches[idx].GetSecondDescription().GetFeature().imageCoordX, matches[idx].GetSecondDescription().GetFeature().imageCoordY), 5, cv::Scalar(0, 255.0, 0.0), 2);
        cv::line(matchImg, pUnscaled[idx], cv::Point2d(pRescaled[idx].x + 1000, pRescaled[idx].y), cv::Scalar(255.0, 0.0, 0.0), 2);
    }

    cv::imshow("Unscaled", matchImg);
    cv::waitKey(0); */
}
