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
        Frame f(std::move(zeros), static_cast<int>(type));

        // Since no image has been provided it should be marked as invalid
        if (type == CV_32F)
        {
            EXPECT_TRUE(f.isValid());
            EXPECT_EQ(f.GetId(), static_cast<int>(type));
            // Image data should have been moved to frame
            EXPECT_TRUE(zeros.empty());
        }
        else
        {
            EXPECT_FALSE(f.isValid());
        }
    }
}
