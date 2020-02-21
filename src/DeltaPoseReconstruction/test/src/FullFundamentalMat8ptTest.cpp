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
#include <FullFundamentalMat8pt.h>

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
TEST(FullFundamentalMat8pt, RotationAndTranslation)
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
    cv::Mat1f KalMat = cv::Mat1f::zeros(3, 3);
    KalMat(0, 0) = 5.0;
    KalMat(1, 1) = 5.0;
    KalMat(2, 2) = 1.0;
    KalMat(0, 2) = centerX;
    KalMat(1, 2) = centerY;
    
    // Projection matrix assuming we are at the center of the world coord. system
    cv::Mat1f trivialProjMat;
    EXPECT_TRUE(GetProjectionMatrix(cv::Mat1f::eye(3, 3), cv::Mat1f::zeros(3, 1), trivialProjMat));

    for (auto coord : realWorldPoints)
    {
        cv::Mat1f projPoint = KalMat * (trivialProjMat * VOCPP::Utils::Point3fToMatHomCoordinates(coord));
        imgPoints.push_back(cv::Point2f(projPoint(0, 0) / projPoint(2, 0), projPoint(1, 0) / projPoint(2, 0)));
    }

    // Projection matrix with nonzero rotation and translation
    cv::Mat1f projMat;
    cv::Mat1f rotMat = GetFrameRotationX(DrawFloatInRange(-0.1F, 0.1F)) * GetFrameRotationY(DrawFloatInRange(-0.1F, 0.1F)) * GetFrameRotationZ(DrawFloatInRange(-0.1F, 0.1F));
    cv::Mat1f translation = (cv::Mat1f(3, 1) << DrawFloatInRange(-1.0F,5.0F), DrawFloatInRange(-1.0F, 5.0F), DrawFloatInRange(-5.0F, 5.0F));   
    EXPECT_TRUE(GetProjectionMatrix(rotMat, translation, projMat));
   
    for (auto coord : realWorldPoints)
    {
        cv::Mat1f projPoint = KalMat * (projMat * VOCPP::Utils::Point3fToMatHomCoordinates(coord));
        scaledImgPoints.push_back(cv::Point2f(projPoint(0, 0) / projPoint(2, 0), projPoint(1, 0) / projPoint(2, 0)));
    }

    // Get fundamental matrix
    std::vector<cv::Mat1f> solutionVec;
    VOCPP::DeltaPoseReconstruction::FullFundamentalMat8pt model;
    model.Compute(scaledImgPoints, imgPoints, solutionVec);
    ASSERT_TRUE(solutionVec.size() == 1);

    // Calculate true one
    cv::Mat1f translatCross;
    GetCrossProductMatrix(translation, translatCross);
    cv::Mat1f trueFundMat =  KalMat.t().inv()* ((translatCross * rotMat)* KalMat.inv());
    trueFundMat = trueFundMat / cv::norm(trueFundMat);
    //They can differ by a global sign
    if (trueFundMat(2, 2) / solutionVec[0](2, 2) < 0.0)
    {
        trueFundMat = -trueFundMat;
    }

    // And check
    EXPECT_NEAR(solutionVec[0](0, 0), trueFundMat(0, 0), 1e-7);
    EXPECT_NEAR(solutionVec[0](0, 1), trueFundMat(0, 1), 1e-7);
    EXPECT_NEAR(solutionVec[0](0, 2), trueFundMat(0, 2), 5e-5);
    EXPECT_NEAR(solutionVec[0](1, 0), trueFundMat(1, 0), 1e-7);
    EXPECT_NEAR(solutionVec[0](1, 1), trueFundMat(1, 1), 1e-7);
    EXPECT_NEAR(solutionVec[0](1, 2), trueFundMat(1, 2), 5e-5);
    EXPECT_NEAR(solutionVec[0](2, 0), trueFundMat(2, 0), 5e-5);
    EXPECT_NEAR(solutionVec[0](2, 1), trueFundMat(2, 1), 5e-5);
    EXPECT_NEAR(solutionVec[0](2, 2), trueFundMat(2, 2), 5e-5);
}
