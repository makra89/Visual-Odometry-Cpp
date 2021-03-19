/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 - 2021 Manuel Kraus
*/

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include<Vocpp_Master/VocppMaster.h>
#include<Vocpp_Calibration/MonoCameraCalibration.h>

// Make sure that it is possible to load a calibration, go out of scope and run the Master
TEST(MasterTest, LoadCalibrationTest)
{
    VOCPP::VocppMaster master;
    
    {
        VOCPP::Calibration::MonoCameraCalibration calib(1.0, 1.0, 1.0, 1.0);
        master.LoadCalibration(calib);
    }

    // Provide an image with five features --> Master should return true
    cv::Mat img = cv::Mat::zeros(500, 500, CV_64F);
    img.at<double>(250, 250) = 1.0;
    img.at<double>(240, 240) = 1.0;
    img.at<double>(260, 260) = 1.0;
    img.at<double>(270, 270) = 1.0;
    img.at<double>(280, 280) = 1.0;
    VOCPP::Frame frame(img, 0U);
    
    EXPECT_TRUE(master.FeedNextFrame(frame));
}

// Make sure that no crash happens when frames with too few features are provided
TEST(MasterTest, TooFewFeatures)
{
    VOCPP::VocppMaster master;
    VOCPP::Calibration::MonoCameraCalibration calib(1.0, 1.0, 1.0, 1.0);
    master.LoadCalibration(calib);

    // Provide empty image --> should return false
    VOCPP::Frame frame1(cv::Mat::zeros(500, 500, CV_64F), 1U);
    EXPECT_FALSE(master.FeedNextFrame(frame1));

    // Another one
    VOCPP::Frame frame2(cv::Mat::zeros(500, 500, CV_64F), 2U);
    EXPECT_FALSE(master.FeedNextFrame(frame2));

    // Provide an image with two features --> too few --> false
    cv::Mat img = cv::Mat::zeros(500, 500, CV_64F);
    img.at<double>(250, 250) = 1.0;
    img.at<double>(320, 320) = 1.0;
    VOCPP::Frame frame3(img, 3U);
    EXPECT_FALSE(master.FeedNextFrame(frame3));
}
