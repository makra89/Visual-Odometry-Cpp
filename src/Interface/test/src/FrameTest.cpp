/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Interface/Frame.h>
#include <gtest/gtest.h>

using VOCPP::Frame;

TEST(FrameTest, Constructor_Empty)
{
    // Construct empty frame
    Frame emptyFrame;

    // Since no image has been provided it should be marked as invalid
    EXPECT_FALSE(emptyFrame.IsValid());
}

TEST(FrameTest, Constructor_NoImageData)
{
    // Construct empty frame
    cv::Mat emptyImage;
    Frame f(NULL, 3, 3, 1);

    // Since no image has been provided it should be marked as invalid
    EXPECT_FALSE(f.IsValid());
}

TEST(FrameTest, Constructor_Valid)
{
    // Creating a valid frame
    float img[] = { 1, 3 ,4 ,5 };    
    Frame f(img, 2, 2, 1);

    EXPECT_TRUE(f.IsValid());
    EXPECT_EQ(f.GetId(), 1);
}
