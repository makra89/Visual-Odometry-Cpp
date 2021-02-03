/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <input.h>

#include <Vocpp_FeatureHandling/OrFastDetector.h>
#include <Vocpp_FeatureHandling/BriefDescriptor.h>
#include <Vocpp_FeatureHandling/BruteForceMatcher.h>
#include <Vocpp_FeatureHandling/LshMatcher.h>


// Test the chain OrFast + Brief + Brute Force
// Test with two shifted triangles
TEST(OrFastChainTest, MatchTriangles_Shifted)
{
    cv::Mat grayScaleImg;
    cv::cvtColor(cv::imread(testDirectory + "Triangle.jpg", 1), grayScaleImg, cv::COLOR_BGR2GRAY);
    grayScaleImg.convertTo(grayScaleImg, CV_32FC1, 1.0 / 255.0);
    // Copy the image to a larger one to shift the image
    cv::Mat img = cv::Mat::ones(1000, 700, CV_32FC1);
    cv::Mat imgShifted = cv::Mat::ones(1000, 700, CV_32FC1);
    grayScaleImg.copyTo(img(cv::Rect(100, 120, grayScaleImg.cols, grayScaleImg.rows)));
    grayScaleImg.copyTo(imgShifted(cv::Rect(65, 55, grayScaleImg.cols, grayScaleImg.rows)));

    VOCPP::FeatureHandling::OrientedFastDetector detector;
    VOCPP::FeatureHandling::BriefDescriptor descriptor;
    VOCPP::FeatureHandling::BruteForceMatcher matcher;

    VOCPP::Frame frame(img.ptr<float>(0), img.cols, img.rows, 1);
    VOCPP::Frame frameShifted(imgShifted.ptr<float>(0), imgShifted.cols, imgShifted.rows, 1);
    std::vector<VOCPP::FeatureHandling::Feature> features;
    std::vector<VOCPP::FeatureHandling::Feature> featuresShifted;
    EXPECT_TRUE(detector.ExtractFeatures(frame, 3U, features));
    EXPECT_TRUE(detector.ExtractFeatures(frameShifted, 3U, featuresShifted));

    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptions;
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptionsShifted;
    EXPECT_TRUE(descriptor.ComputeDescriptions(frame, features, descriptions));
    EXPECT_TRUE(descriptor.ComputeDescriptions(frameShifted, featuresShifted, descriptionsShifted));

    std::vector<VOCPP::FeatureHandling::BinaryDescriptionMatch> matches;
    EXPECT_TRUE(matcher.MatchDesriptions(descriptions, descriptionsShifted, matches));
    
    EXPECT_EQ(3U, matches.size());
    for (unsigned int idx = 0U; idx < matches.size(); idx++)
    {
        float posXdiff = matches[idx].GetFirstFeature().imageCoordX - matches[idx].GetSecondFeature().imageCoordX;
        EXPECT_EQ(posXdiff, 35);
        float posYdiff = matches[idx].GetFirstFeature().imageCoordY - matches[idx].GetSecondFeature().imageCoordY;
        EXPECT_EQ(posYdiff, 65);

        // We expect that the feature angles do not differ
        float angleDiff = std::abs(matches[idx].GetFirstFeature().angle - matches[idx].GetSecondFeature().angle);
        EXPECT_NEAR(angleDiff, 0.f, 0.001f);
    }
}

// Test the chain OrFast + Brief + Lsh
// Test with two shifted triangles
TEST(OrFastChainTest_Lsh, MatchTriangles_Shifted)
{
    cv::Mat grayScaleImg;
    cv::cvtColor(cv::imread(testDirectory + "Triangle.jpg", 1), grayScaleImg, cv::COLOR_BGR2GRAY);
    grayScaleImg.convertTo(grayScaleImg, CV_32FC1, 1.0 / 255.0);
    // Copy the image to a larger one to shift the image
    cv::Mat img = cv::Mat::ones(1000, 700, CV_32FC1);
    cv::Mat imgShifted = cv::Mat::ones(1000, 700, CV_32FC1);
    grayScaleImg.copyTo(img(cv::Rect(100, 120, grayScaleImg.cols, grayScaleImg.rows)));
    grayScaleImg.copyTo(imgShifted(cv::Rect(65, 55, grayScaleImg.cols, grayScaleImg.rows)));

    VOCPP::FeatureHandling::OrientedFastDetector detector;
    VOCPP::FeatureHandling::BriefDescriptor descriptor;
    VOCPP::FeatureHandling::LshMatcher matcher;

    VOCPP::Frame frame(img.ptr<float>(0), img.cols, img.rows, 1);
    VOCPP::Frame frameShifted(imgShifted.ptr<float>(0), imgShifted.cols, imgShifted.rows, 1);
    std::vector<VOCPP::FeatureHandling::Feature> features;
    std::vector<VOCPP::FeatureHandling::Feature> featuresShifted;
    EXPECT_TRUE(detector.ExtractFeatures(frame, 3U, features));
    EXPECT_TRUE(detector.ExtractFeatures(frameShifted, 3U, featuresShifted));

    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptions;
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptionsShifted;
    EXPECT_TRUE(descriptor.ComputeDescriptions(frame, features, descriptions));
    EXPECT_TRUE(descriptor.ComputeDescriptions(frameShifted, featuresShifted, descriptionsShifted));

    std::vector<VOCPP::FeatureHandling::BinaryDescriptionMatch> matches;
    EXPECT_TRUE(matcher.MatchDesriptions(descriptions, descriptionsShifted, matches));

    // One edge is dropped by LSH
    EXPECT_EQ(2U, matches.size());
    for (unsigned int idx = 0U; idx < matches.size(); idx++)
    {
        float posXdiff = matches[idx].GetFirstFeature().imageCoordX - matches[idx].GetSecondFeature().imageCoordX;
        EXPECT_EQ(posXdiff, 35);
        float posYdiff = matches[idx].GetFirstFeature().imageCoordY - matches[idx].GetSecondFeature().imageCoordY;
        EXPECT_EQ(posYdiff, 65);

        // We expect that the feature angles do not differ
        float angleDiff = std::abs(matches[idx].GetFirstFeature().angle - matches[idx].GetSecondFeature().angle);
        EXPECT_NEAR(angleDiff, 0.f, 0.001f);
    }
}

// Test the chain OrFast + Brief + Brute Force
// Test rotation invariance using rotated images (rotated by 90 degrees)
TEST(OrFastChainTest, MatchTriangles_Rotated)
{
    cv::Mat grayScaleImg;
    cv::Mat grayScaleImgRotated;
    cv::cvtColor(cv::imread(testDirectory + "far-north.jpg", 1), grayScaleImg, cv::COLOR_BGR2GRAY);
    grayScaleImg.convertTo(grayScaleImg, CV_32FC1, 1.0 / 255.0);
    cv::rotate(grayScaleImg, grayScaleImgRotated, cv::ROTATE_90_CLOCKWISE);

    VOCPP::FeatureHandling::OrientedFastDetector detector;
    VOCPP::FeatureHandling::BriefDescriptor descriptor;
    VOCPP::FeatureHandling::BruteForceMatcher matcher;

    VOCPP::Frame frame(grayScaleImg.ptr<float>(0), grayScaleImg.cols, grayScaleImg.rows, 1);
    VOCPP::Frame frameRotated(grayScaleImgRotated.ptr<float>(0), grayScaleImgRotated.cols, grayScaleImgRotated.rows, 1);
    std::vector<VOCPP::FeatureHandling::Feature> features;
    std::vector<VOCPP::FeatureHandling::Feature> featuresRotated;
    EXPECT_TRUE(detector.ExtractFeatures(frame, 500U, features));
    EXPECT_TRUE(detector.ExtractFeatures(frameRotated, 500U, featuresRotated));

    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptions;
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptionsRotated;
    EXPECT_TRUE(descriptor.ComputeDescriptions(frame, features, descriptions));
    EXPECT_TRUE(descriptor.ComputeDescriptions(frameRotated, featuresRotated, descriptionsRotated));

    std::vector<VOCPP::FeatureHandling::BinaryDescriptionMatch> matches;
    EXPECT_TRUE(matcher.MatchDesriptions(descriptions, descriptionsRotated, matches));

    // We requested 500 features, not all features will be matched
    EXPECT_GE(matches.size(), 495U);
    
    // Since we rotated by 90 degrees, we know the exact matching pixel locations
    for (unsigned int idx = 0U; idx < matches.size(); idx++)
    {
        EXPECT_EQ(matches[idx].GetFirstFeature().imageCoordX, matches[idx].GetSecondFeature().imageCoordY);
        EXPECT_EQ(matches[idx].GetFirstFeature().imageCoordY, (grayScaleImg.rows - 1U) - matches[idx].GetSecondFeature().imageCoordX);

        // We expect that the feature angle differ by pi/2
        float angleDiff = std::abs(matches[idx].GetFirstFeature().angle - matches[idx].GetSecondFeature().angle);
        if (angleDiff > CV_PI) angleDiff -= static_cast<float>(CV_PI);
        EXPECT_NEAR(angleDiff, CV_PI / 2., 0.2);
    }
}


// Test the chain OrFast + Brief + Brute Force
// Test rotation invariance using rotated images (rotated by 90 degrees)
TEST(OrFastChainTest_Lsh, MatchTriangles_Rotated)
{
    cv::Mat grayScaleImg;
    cv::Mat grayScaleImgRotated;
    cv::cvtColor(cv::imread(testDirectory + "far-north.jpg", 1), grayScaleImg, cv::COLOR_BGR2GRAY);
    grayScaleImg.convertTo(grayScaleImg, CV_32FC1, 1.0 / 255.0);
    cv::rotate(grayScaleImg, grayScaleImgRotated, cv::ROTATE_90_CLOCKWISE);

    VOCPP::FeatureHandling::OrientedFastDetector detector;
    VOCPP::FeatureHandling::BriefDescriptor descriptor;
    VOCPP::FeatureHandling::LshMatcher matcher;

    VOCPP::Frame frame(grayScaleImg.ptr<float>(0), grayScaleImg.cols, grayScaleImg.rows, 1);
    VOCPP::Frame frameRotated(grayScaleImgRotated.ptr<float>(0), grayScaleImgRotated.cols, grayScaleImgRotated.rows, 1);
    std::vector<VOCPP::FeatureHandling::Feature> features;
    std::vector<VOCPP::FeatureHandling::Feature> featuresRotated;
    EXPECT_TRUE(detector.ExtractFeatures(frame, 500U, features));
    EXPECT_TRUE(detector.ExtractFeatures(frameRotated, 500U, featuresRotated));

    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptions;
    std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription> descriptionsRotated;
    EXPECT_TRUE(descriptor.ComputeDescriptions(frame, features, descriptions));
    EXPECT_TRUE(descriptor.ComputeDescriptions(frameRotated, featuresRotated, descriptionsRotated));

    std::vector<VOCPP::FeatureHandling::BinaryDescriptionMatch> matches;
    EXPECT_TRUE(matcher.MatchDesriptions(descriptions, descriptionsRotated, matches));

    // We requested 500 features, not all features will be matched
    EXPECT_GE(matches.size(), 400U);

    // Since we rotated by 90 degrees, we know the exact matching pixel locations
    for (unsigned int idx = 0U; idx < matches.size(); idx++)
    {
        EXPECT_EQ(matches[idx].GetFirstFeature().imageCoordX, matches[idx].GetSecondFeature().imageCoordY);
        EXPECT_EQ(matches[idx].GetFirstFeature().imageCoordY, (grayScaleImg.rows - 1U) - matches[idx].GetSecondFeature().imageCoordX);

        // We expect that the feature angle differ by pi/2
        float angleDiff = std::abs(matches[idx].GetFirstFeature().angle - matches[idx].GetSecondFeature().angle);
        if (angleDiff > CV_PI) angleDiff -= static_cast<float>(CV_PI);
        EXPECT_NEAR(angleDiff, CV_PI / 2., 0.2);
    }
}
