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
    Frame fDouble((double*)nullptr, 3, 3, 1);

    // Since no image has been provided it should be marked as invalid
    EXPECT_FALSE(fDouble.IsValid());

    // Construct empty frame
    Frame fUint((uint8_t*)nullptr, 3, 3, 1);

    // Since no image has been provided it should be marked as invalid
    EXPECT_FALSE(fUint.IsValid());
}

TEST(FrameTest, Constructor_Valid_Double)
{
    // Creating a valid frame
    double img[] = {0., 1.};    
    Frame f(img, 1, 2, 1);

    EXPECT_TRUE(f.IsValid());
    EXPECT_EQ(f.GetId(), 1);

    EXPECT_EQ(f.GetImage().at<double>(0, 0), 0);
    EXPECT_EQ(f.GetImage().at<double>(0, 1), 1);

}

TEST(FrameTest, Constructor_Valid_Uint)
{
    // Creating a valid frame
    uint8_t img[] = { 0, 255, 1};
    Frame f(img, 3, 1, 1);

    EXPECT_TRUE(f.IsValid());
    EXPECT_EQ(f.GetId(), 1);

    EXPECT_EQ(f.GetImage().at<double>(0, 0), 0);
    EXPECT_EQ(f.GetImage().at<double>(1, 0), 1);
    EXPECT_EQ(f.GetImage().at<double>(2, 0), 1./255.);
}
