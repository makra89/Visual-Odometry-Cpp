/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/Frame.h>
#include <gtest/gtest.h>

using VOCPP::Utils::Frame;

TEST(FrameTest, Constructor_Empty)
{
    // Construct empty frame
    Frame emptyFrame;

    // Since no image has been provided it should be marked as invalid
    EXPECT_FALSE(emptyFrame.isValid());
}


TEST(FrameTest, Constructor_NoImageData)
{
    // Construct empty frame
    cv::Mat emptyImage;
    Frame f(std::move(emptyImage), 0U);

    // Since no image has been provided it should be marked as invalid
    EXPECT_FALSE(f.isValid());
}

TEST(FrameTest, Constructor_SupportedImageTypes)
{
    // Construct empty frame
    for (int type = 0; type < 30; type++)
    {
        cv::Mat zeros = cv::Mat::zeros(4, 4, type);
        Frame f(std::move(zeros), static_cast<uint32_t>(type));

        // Since no image has been provided it should be marked as invalid
        if (type == CV_32F)
        {
            EXPECT_TRUE(f.isValid());
            EXPECT_EQ(f.GetId(), static_cast<uint32_t>(type));
            // Image data should have been moved to frame
            EXPECT_TRUE(zeros.empty());
        }
        else
        {
            EXPECT_FALSE(f.isValid());
        }
    }
}

TEST(FrameTest, SetAndGetKeypoints)
{
    // Create keypoint vector with one element
    std::vector<cv::KeyPoint> keyVec;
    keyVec.push_back(cv::KeyPoint(cv::Point2f(0.0, 0.0), 1.0));
    EXPECT_EQ(keyVec.size(), 1U);

    // Create frame and set keypoints
    cv::Mat zeros = cv::Mat::zeros(4, 4, CV_32F);
    Frame f(std::move(zeros), 0U);
    f.SetKeypoints(std::move(keyVec));

    // keypoint vector should have been moved to frame
    EXPECT_EQ(keyVec.size(), 0U);
    EXPECT_EQ(f.GetKeypoints().size(), 1U);
}

TEST(FrameTest, SetAndGetDescriptions)
{
    // Create description vector with one element
    std::vector<cv::Mat> descVec;
    descVec.push_back(cv::Mat::zeros(2,2,CV_32F));
    EXPECT_EQ(descVec.size(), 1U);

    // Create frame and set descriptions
    cv::Mat zeros = cv::Mat::zeros(4, 4, CV_32F);
    Frame f(std::move(zeros), 0U);
    f.SetDescriptions(std::move(descVec));

    // description vector should have been moved to frame
    EXPECT_EQ(descVec.size(), 0U);
    EXPECT_EQ(f.GetDescriptions().size(), 1U);
}

TEST(FrameTest, SetAndGetMatches)
{
    // Create match vector with one element
    std::vector<cv::DMatch> matchVec;
    matchVec.push_back(cv::DMatch(1, 1, 1.0));
    EXPECT_EQ(matchVec.size(), 1U);

    // Create frame and set matches
    cv::Mat zeros = cv::Mat::zeros(4, 4, CV_32F);
    Frame f(std::move(zeros), 0U);
    f.SetMatches(std::move(matchVec));

    // match vector should have been moved to frame
    EXPECT_EQ(matchVec.size(), 0U);
    EXPECT_EQ(f.GetMatches().size(), 1U);
}
