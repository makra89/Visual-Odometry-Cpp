/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/EpipolarGeometryUtils.h>
#include <Vocpp_Utils/ConversionUtils.h>
#include <Vocpp_Utils/NumericalUtilities.h>
#include <Vocpp_Utils/FrameRotations.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

using VOCPP::Utils::DrawFloatInRange;
using VOCPP::Utils::GetFrameRotationX;
using VOCPP::Utils::GetFrameRotationY;
using VOCPP::Utils::GetFrameRotationZ;
using VOCPP::Utils::GetProjectionMatrix;
using VOCPP::Utils::GetCrossProductMatrix;

/* Most general type of fundamental matrix consisting out of a translation + rotation 
If additionally the point cloud lies not on a plane we can be sure that for this kind 
of movement we don't have any kind of degeneracy */
TEST(FundamentalMatrix8pt, RotationAndTranslation)
{
    std::vector<cv::Point3f> realWorldPoints;
    std::vector<cv::Point2f> imgPoints;
    std::vector<cv::Point2f> scaledImgPoints;

    const double centerX = 400.0;
    const double centerY = 600.0;

    // Construct set of 3D points in world coordinate system
    const float minDist = 150.0;
    for (int it = 0; it < 12; it++)
    {
        realWorldPoints.push_back(cv::Point3f(DrawFloatInRange(-minDist, minDist) , DrawFloatInRange(-minDist, minDist), DrawFloatInRange(-5.0, 5.0)));
    }

    // Kalibration matrix with shifted image center
    cv::Mat KalMat = cv::Mat::zeros(3, 3, CV_32F);
    KalMat.at<float>(0, 0) = 5.0;
    KalMat.at<float>(1, 1) = 5.0;
    KalMat.at<float>(2, 2) = 1.0;
    KalMat.at<float>(0, 2) = centerX;
    KalMat.at<float>(1, 2) = centerY;
    
    // Projection matrix assuming we are at the center of the world coord. system
    cv::Mat trivialProjMat;
    EXPECT_TRUE(GetProjectionMatrix(cv::Mat::eye(3, 3, CV_32F), cv::Mat::zeros(3, 1, CV_32F), trivialProjMat));

    for (auto coord : realWorldPoints)
    {
        cv::Mat projPoint = KalMat * (trivialProjMat * VOCPP::Utils::Point3fToMatHomCoordinates(coord));
        imgPoints.push_back(cv::Point2f(projPoint.at<float>(0, 0) / projPoint.at<float>(2, 0), projPoint.at<float>(1, 0) / projPoint.at<float>(2, 0)));
    }

    // Projection matrix with nonzero rotation and translation
    cv::Mat projMat;
    cv::Mat rotMat = GetFrameRotationX(DrawFloatInRange(-0.1, 0.1)) * GetFrameRotationY(DrawFloatInRange(-0.1, 0.1)) * GetFrameRotationZ(DrawFloatInRange(-0.1, 0.1));
    cv::Mat translation = (cv::Mat_<float>(3, 1) << DrawFloatInRange(-1.0,5.0), DrawFloatInRange(-1.0, 5.0), DrawFloatInRange(-5.0, 5.0));   
    EXPECT_TRUE(GetProjectionMatrix(rotMat, translation, projMat));
   
    for (auto coord : realWorldPoints)
    {
        cv::Mat projPoint = KalMat * (projMat * VOCPP::Utils::Point3fToMatHomCoordinates(coord));
        scaledImgPoints.push_back(cv::Point2f(projPoint.at<float>(0, 0) / projPoint.at<float>(2, 0), projPoint.at<float>(1, 0) / projPoint.at<float>(2, 0)));
    }

    // Get fundamental matrix 
    cv::Mat fundMatrix;
    VOCPP::Utils::CalculateFundamentalMatrix8pt(scaledImgPoints, imgPoints, fundMatrix);

    // Calculate true one
    cv::Mat translatCross;
    GetCrossProductMatrix(translation, translatCross);
    cv::Mat trueFundMat =  KalMat.t().inv()* ((translatCross * rotMat)* KalMat.inv());
    trueFundMat = trueFundMat / trueFundMat.at<float>(2, 2);
   
    // And check
    EXPECT_NEAR(fundMatrix.at<float>(0, 0), trueFundMat.at<float>(0, 0), 1e-7);
    EXPECT_NEAR(fundMatrix.at<float>(0, 1), trueFundMat.at<float>(0, 1), 1e-7);
    EXPECT_NEAR(fundMatrix.at<float>(0, 2), trueFundMat.at<float>(0, 2), 5e-5);
    EXPECT_NEAR(fundMatrix.at<float>(1, 0), trueFundMat.at<float>(1, 0), 1e-7);
    EXPECT_NEAR(fundMatrix.at<float>(1, 1), trueFundMat.at<float>(1, 1), 1e-7);
    EXPECT_NEAR(fundMatrix.at<float>(1, 2), trueFundMat.at<float>(1, 2), 5e-5);
    EXPECT_NEAR(fundMatrix.at<float>(2, 0), trueFundMat.at<float>(2, 0), 5e-5);
    EXPECT_NEAR(fundMatrix.at<float>(2, 1), trueFundMat.at<float>(2, 1), 5e-5);
    EXPECT_NEAR(fundMatrix.at<float>(2, 2), trueFundMat.at<float>(2, 2), 5e-5);
}
