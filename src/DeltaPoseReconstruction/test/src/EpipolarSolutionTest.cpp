/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <EpipolarSolution.h>
#include <Vocpp_Utils/ConversionUtils.h>
#include <Vocpp_Utils/NumericalUtilities.h>
#include <Vocpp_Utils/FrameRotations.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

using VOCPP::Utils::DrawDoubleInRange;
using VOCPP::Utils::GetFrameRotationX;
using VOCPP::Utils::GetFrameRotationY;
using VOCPP::Utils::GetFrameRotationZ;
using VOCPP::Utils::GetCrossProductMatrix;
using VOCPP::Utils::ImageProjectionMatrix;


TEST(RansacTest, TranslationAndRotation)
{
    for (uint32_t testIt = 0U; testIt < 10U; testIt++)
    {
        std::vector<cv::Point3d> realWorldPoints;
        std::vector<cv::Point2d> imgPoints;
        std::vector<cv::Point2d> scaledImgPoints;

        const double centerX = 400.F;
        const double centerY = 600.F;

        // Construct set of 3D points in world coordinate system
        // Take 100 points
        const double maxDist = 3.0;
        for (uint32_t it = 0U; it < 100U; it++)
        {
            realWorldPoints.push_back(cv::Point3d(DrawDoubleInRange(-maxDist, maxDist), DrawDoubleInRange(-maxDist, maxDist), DrawDoubleInRange(4.0, 7.0)));
        }

        // Kalibration matrix with shifted image center
        cv::Mat1d calMat = cv::Mat1d::zeros(3, 3);
        calMat(0, 0) = 100.F;
        calMat(1, 1) = 100.F;
        calMat(2, 2) = 1.F;
        calMat(0, 2) = centerX;
        calMat(1, 2) = centerY;

        // Projection matrix assuming we are at the center of the world coord. system
        ImageProjectionMatrix trivialProjMat(cv::Mat1d::eye(3, 3), cv::Mat1d::zeros(3, 1), calMat);

        for (auto coord : realWorldPoints)
        {
            imgPoints.push_back(trivialProjMat.Apply(coord));
        }

        // Projection matrix with nonzero rotation and translation
        const double angleRange = 0.2F;
        cv::Mat1d rotTrue = GetFrameRotationX(DrawDoubleInRange(-angleRange, angleRange)) * GetFrameRotationY(DrawDoubleInRange(-angleRange, angleRange)) * GetFrameRotationZ(DrawDoubleInRange(-angleRange, angleRange));
        cv::Mat1d translationTrue = (cv::Mat1d(3, 1) << DrawDoubleInRange(-2.0F, 2.0F), DrawDoubleInRange(-2.0F, 2.0F), DrawDoubleInRange(-2.0F, 2.0F));
        ImageProjectionMatrix projMat(rotTrue, translationTrue, calMat);
        // Actually we want to know the translation from scaled -> img 
        // in the scaled coordinate frame --> Apply transformation
        translationTrue = -(rotTrue.t() * translationTrue);

        for (auto coord : realWorldPoints)
        {
            scaledImgPoints.push_back(projMat.Apply(coord));
        }

        std::vector<uint32_t> inlierIndices;
        cv::Mat1d translation;
        cv::Mat1d rotation;
        std::vector<cv::Point3d> triangulatedPoints;
        EXPECT_TRUE(VOCPP::DeltaPoseReconstruction::RecoverPoseRansac(scaledImgPoints, imgPoints, calMat, inlierIndices, translation, rotation, triangulatedPoints));        
        // Expect that all matches are flagged as inliers
        EXPECT_EQ(100, inlierIndices.size());

        EXPECT_NEAR(rotTrue(0, 0), rotation(0, 0), 1e-3);
        EXPECT_NEAR(rotTrue(0, 1), rotation(0, 1), 1e-3);
        EXPECT_NEAR(rotTrue(0, 2), rotation(0, 2), 1e-3);
        EXPECT_NEAR(rotTrue(1, 0), rotation(1, 0), 1e-3);
        EXPECT_NEAR(rotTrue(1, 1), rotation(1, 1), 1e-3);
        EXPECT_NEAR(rotTrue(1, 2), rotation(1, 2), 1e-3);
        EXPECT_NEAR(rotTrue(2, 0), rotation(2, 0), 1e-3);
        EXPECT_NEAR(rotTrue(2, 1), rotation(2, 1), 1e-3);
        EXPECT_NEAR(rotTrue(2, 2), rotation(2, 2), 1e-3);

        // Translation is only known up to a global scale
        translationTrue = translationTrue / cv::norm(translationTrue);
        if (translationTrue(0, 0) / translation(0, 0) < 0.)
        {
            translation = -translation;
        }

        EXPECT_NEAR(translationTrue(0, 0), translation(0, 0), 5e-3);
        EXPECT_NEAR(translationTrue(1, 0), translation(1, 0), 5e-3);
        EXPECT_NEAR(translationTrue(2, 0), translation(2, 0), 5e-3);
    }
}

TEST(RansacTest, TranslationAndRotation_WithOutliers)
{
    for (uint32_t testIt = 0U; testIt < 10U; testIt++)
    {
        std::vector<cv::Point3d> realWorldPoints;
        std::vector<cv::Point2d> imgPoints;
        std::vector<cv::Point2d> scaledImgPoints;

        const double centerX = 400.F;
        const double centerY = 600.F;

        // Construct set of 3D points in world coordinate system
        // Take 100 points
        const double maxDist = 3.0;
        for (uint32_t it = 0; it < 100U; it++)
        {
            realWorldPoints.push_back(cv::Point3d(DrawDoubleInRange(-maxDist, maxDist), DrawDoubleInRange(-maxDist, maxDist), DrawDoubleInRange(4.0, 7.0)));
        }

        // Kalibration matrix with shifted image center
        cv::Mat1d calMat = cv::Mat1d::zeros(3, 3);
        calMat(0, 0) = 100.F;
        calMat(1, 1) = 100.F;
        calMat(2, 2) = 1.F;
        calMat(0, 2) = centerX;
        calMat(1, 2) = centerY;

        // Projection matrix assuming we are at the center of the world coord. system
        ImageProjectionMatrix trivialProjMat(cv::Mat1d::eye(3, 3), cv::Mat1d::zeros(3, 1), calMat);

        for (auto coord : realWorldPoints)
        {
            imgPoints.push_back(trivialProjMat.Apply(coord));
        }

        // Projection matrix with nonzero rotation and translation
        const double angleRange = 0.2F;
        cv::Mat1d rotTrue = GetFrameRotationX(DrawDoubleInRange(-angleRange, angleRange)) * GetFrameRotationY(DrawDoubleInRange(-angleRange, angleRange)) * GetFrameRotationZ(DrawDoubleInRange(-angleRange, angleRange));
        cv::Mat1d translationTrue = (cv::Mat1d(3, 1) << DrawDoubleInRange(-2.0F, 2.0F), DrawDoubleInRange(-2.0F, 2.0F), DrawDoubleInRange(-2.0F, 2.0F));
        ImageProjectionMatrix projMat(rotTrue, translationTrue, calMat);
        // Actually we want to know the translation from scaled -> img 
        // in the scaled coordinate frame --> Apply transformation
        translationTrue = -(rotTrue.t() * translationTrue);

        for (auto coord : realWorldPoints)
        {
            scaledImgPoints.push_back(projMat.Apply(coord));
        }

        // Add some outliers
        for (uint32_t outlierIt = 0U; outlierIt < 10U; outlierIt++)
        {
            imgPoints.push_back(cv::Point2d(DrawDoubleInRange(-100.F, 100.F), DrawDoubleInRange(-100.F, 100.F)));
            scaledImgPoints.push_back(cv::Point2d(DrawDoubleInRange(-100.F, 100.F), DrawDoubleInRange(-100.F, 100.F)));
        }

        std::vector<uint32_t> inlierIndices;
        cv::Mat1d translation;
        cv::Mat1d rotation;
        std::vector<cv::Point3d> triangulatedPoints;
        EXPECT_TRUE(VOCPP::DeltaPoseReconstruction::RecoverPoseRansac(scaledImgPoints, imgPoints, calMat, inlierIndices, translation, rotation, triangulatedPoints));
        EXPECT_GE(inlierIndices.size(), 95);

        EXPECT_NEAR(rotTrue(0, 0), rotation(0, 0), 5e-2);
        EXPECT_NEAR(rotTrue(0, 1), rotation(0, 1), 5e-2);
        EXPECT_NEAR(rotTrue(0, 2), rotation(0, 2), 5e-2);
        EXPECT_NEAR(rotTrue(1, 0), rotation(1, 0), 5e-2);
        EXPECT_NEAR(rotTrue(1, 1), rotation(1, 1), 5e-2);
        EXPECT_NEAR(rotTrue(1, 2), rotation(1, 2), 5e-2);
        EXPECT_NEAR(rotTrue(2, 0), rotation(2, 0), 5e-2);
        EXPECT_NEAR(rotTrue(2, 1), rotation(2, 1), 5e-2);
        EXPECT_NEAR(rotTrue(2, 2), rotation(2, 2), 5e-2);

        // Translation is only known up to a global scale
        translationTrue = translationTrue / cv::norm(translationTrue);
        if (translationTrue(0, 0) / translation(0, 0) < 0.)
        {
            translation = -translation;
        }

        EXPECT_NEAR(translationTrue(0, 0), translation(0, 0), 5e-2);
        EXPECT_NEAR(translationTrue(1, 0), translation(1, 0), 5e-2);
        EXPECT_NEAR(translationTrue(2, 0), translation(2, 0), 5e-2);
    }
}
