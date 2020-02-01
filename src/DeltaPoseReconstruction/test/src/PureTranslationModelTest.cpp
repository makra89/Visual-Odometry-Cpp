/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/ConversionUtils.h>
#include <Vocpp_Utils/NumericalUtilities.h>
#include <Vocpp_Utils/FrameRotations.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <EpipolarModel.h>
#include <PureTranslationModel.h>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

using VOCPP::Utils::DrawFloatInRange;
using VOCPP::Utils::GetFrameRotationX;
using VOCPP::Utils::GetFrameRotationY;
using VOCPP::Utils::GetFrameRotationZ;
using VOCPP::Utils::GetProjectionMatrix;
using VOCPP::Utils::GetCrossProductMatrix;

/* We restrict the fundamental matrix to translation */
TEST(PureTranslationModel, Translation)
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

    // Projection matrix with translation only
    cv::Mat projMat;
    cv::Mat translation = (cv::Mat_<float>(3, 1) << DrawFloatInRange(-1.0,5.0), DrawFloatInRange(-1.0, 5.0), DrawFloatInRange(-5.0, 5.0));   
    EXPECT_TRUE(GetProjectionMatrix(cv::Mat::eye(3, 3, CV_32F), translation, projMat));
   
    for (auto coord : realWorldPoints)
    {
        cv::Mat projPoint = KalMat * (projMat * VOCPP::Utils::Point3fToMatHomCoordinates(coord));
        scaledImgPoints.push_back(cv::Point2f(projPoint.at<float>(0, 0) / projPoint.at<float>(2, 0), projPoint.at<float>(1, 0) / projPoint.at<float>(2, 0)));
    }

    // Get fundamental matrix
    std::vector<cv::Mat> solutionVec;
    VOCPP::DeltaPoseReconstruction::PureTranslationModel model;
    model.Compute(scaledImgPoints, imgPoints, solutionVec);
    ASSERT_TRUE(solutionVec.size() == 1);

    // Calculate true one
    cv::Mat translatCross;
    GetCrossProductMatrix(translation, translatCross);
    cv::Mat trueFundMat =  KalMat.t().inv()* ((translatCross * cv::Mat::eye(3, 3, CV_32F))* KalMat.inv());
    trueFundMat = trueFundMat / cv::norm(trueFundMat);
    //They can differ by a global sign
    if (trueFundMat.at<float>(0, 2) / solutionVec[0].at<float>(0, 2) < 0.0)
    {
        trueFundMat = -trueFundMat;
    }

    // And check
    EXPECT_NEAR(solutionVec[0].at<float>(0, 0), trueFundMat.at<float>(0, 0), 1e-7);
    EXPECT_NEAR(solutionVec[0].at<float>(0, 1), trueFundMat.at<float>(0, 1), 1e-7);
    EXPECT_NEAR(solutionVec[0].at<float>(0, 2), trueFundMat.at<float>(0, 2), 5e-5);
    EXPECT_NEAR(solutionVec[0].at<float>(1, 0), trueFundMat.at<float>(1, 0), 1e-7);
    EXPECT_NEAR(solutionVec[0].at<float>(1, 1), trueFundMat.at<float>(1, 1), 1e-7);
    EXPECT_NEAR(solutionVec[0].at<float>(1, 2), trueFundMat.at<float>(1, 2), 5e-5);
    EXPECT_NEAR(solutionVec[0].at<float>(2, 0), trueFundMat.at<float>(2, 0), 5e-5);
    EXPECT_NEAR(solutionVec[0].at<float>(2, 1), trueFundMat.at<float>(2, 1), 5e-5);
    EXPECT_NEAR(solutionVec[0].at<float>(2, 2), trueFundMat.at<float>(2, 2), 5e-5);
}
