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

using VOCPP::Utils::DrawFloatInRange;
using VOCPP::Utils::GetFrameRotationX;
using VOCPP::Utils::GetFrameRotationY;
using VOCPP::Utils::GetFrameRotationZ;
using VOCPP::Utils::GetCrossProductMatrix;
using VOCPP::Utils::ImageProjectionMatrix;


TEST(RansacTest, TranslationAndRotation)
{
    for (int testIt = 0; testIt < 10; testIt++)
    {
        std::vector<cv::Point3f> realWorldPoints;
        std::vector<cv::Point2f> imgPoints;
        std::vector<cv::Point2f> scaledImgPoints;

        const double centerX = 400.F;
        const double centerY = 600.F;

        // Construct set of 3D points in world coordinate system
        // Take 100 points
        const float maxDist = 3.0;
        for (int it = 0; it < 100; it++)
        {
            realWorldPoints.push_back(cv::Point3f(DrawFloatInRange(-maxDist, maxDist), DrawFloatInRange(-maxDist, maxDist), DrawFloatInRange(4.0, 7.0)));
        }

        // Kalibration matrix with shifted image center
        cv::Mat1f calMat = cv::Mat1f::zeros(3, 3);
        calMat(0, 0) = 100.F;
        calMat(1, 1) = 100.F;
        calMat(2, 2) = 1.F;
        calMat(0, 2) = centerX;
        calMat(1, 2) = centerY;

        // Projection matrix assuming we are at the center of the world coord. system
        ImageProjectionMatrix trivialProjMat(cv::Mat1f::eye(3, 3), cv::Mat1f::zeros(3, 1), calMat);

        for (auto coord : realWorldPoints)
        {
            imgPoints.push_back(trivialProjMat.Apply(coord));
        }

        // Projection matrix with nonzero rotation and translation
        const float angleRange = 0.2F;
        cv::Mat1f rotTrue = GetFrameRotationX(DrawFloatInRange(-angleRange, angleRange)) * GetFrameRotationY(DrawFloatInRange(-angleRange, angleRange)) * GetFrameRotationZ(DrawFloatInRange(-angleRange, angleRange));
        cv::Mat1f translationTrue = (cv::Mat1f(3, 1) << DrawFloatInRange(-2.0F, 2.0F), DrawFloatInRange(-2.0F, 2.0F), DrawFloatInRange(-2.0F, 2.0F));
        ImageProjectionMatrix projMat(rotTrue, translationTrue, calMat);
        // Actually we want to know the translation from scaled -> img 
        // in the scaled coordinate frame --> Apply transformation
        translationTrue = -(rotTrue.t() * translationTrue);

        for (auto coord : realWorldPoints)
        {
            scaledImgPoints.push_back(projMat.Apply(coord));
        }

        std::vector<unsigned int> inlierIndices;
        cv::Mat1f translation;
        cv::Mat1f rotation;

        EXPECT_TRUE(VOCPP::DeltaPoseReconstruction::RecoverPoseRansac(scaledImgPoints, imgPoints, calMat, inlierIndices, translation, rotation));
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

// Disabled since currently Ransac is not so robust against outliers --> improve and activate
TEST(RansacTest, TranslationAndRotation_WithOutliers)
{
    for (int testIt = 0; testIt < 100; testIt++)
    {
        std::vector<cv::Point3f> realWorldPoints;
        std::vector<cv::Point2f> imgPoints;
        std::vector<cv::Point2f> scaledImgPoints;

        const double centerX = 400.F;
        const double centerY = 600.F;

        // Construct set of 3D points in world coordinate system
        // Take 100 points
        const float maxDist = 3.0;
        for (int it = 0; it < 100; it++)
        {
            realWorldPoints.push_back(cv::Point3f(DrawFloatInRange(-maxDist, maxDist), DrawFloatInRange(-maxDist, maxDist), DrawFloatInRange(4.0, 7.0)));
        }

        // Kalibration matrix with shifted image center
        cv::Mat1f calMat = cv::Mat1f::zeros(3, 3);
        calMat(0, 0) = 100.F;
        calMat(1, 1) = 100.F;
        calMat(2, 2) = 1.F;
        calMat(0, 2) = centerX;
        calMat(1, 2) = centerY;

        // Projection matrix assuming we are at the center of the world coord. system
        ImageProjectionMatrix trivialProjMat(cv::Mat1f::eye(3, 3), cv::Mat1f::zeros(3, 1), calMat);

        for (auto coord : realWorldPoints)
        {
            imgPoints.push_back(trivialProjMat.Apply(coord));
        }

        // Projection matrix with nonzero rotation and translation
        const float angleRange = 0.2F;
        cv::Mat1f rotTrue = GetFrameRotationX(DrawFloatInRange(-angleRange, angleRange)) * GetFrameRotationY(DrawFloatInRange(-angleRange, angleRange)) * GetFrameRotationZ(DrawFloatInRange(-angleRange, angleRange));
        cv::Mat1f translationTrue = (cv::Mat1f(3, 1) << DrawFloatInRange(-2.0F, 2.0F), DrawFloatInRange(-2.0F, 2.0F), DrawFloatInRange(-2.0F, 2.0F));
        ImageProjectionMatrix projMat(rotTrue, translationTrue, calMat);
        // Actually we want to know the translation from scaled -> img 
        // in the scaled coordinate frame --> Apply transformation
        translationTrue = -(rotTrue.t() * translationTrue);

        for (auto coord : realWorldPoints)
        {
            scaledImgPoints.push_back(projMat.Apply(coord));
        }

        // Add some outliers
        for (int outlierIt = 0; outlierIt < 10; outlierIt++)
        {
            imgPoints.push_back(cv::Point2f(DrawFloatInRange(-100.F, 100.F), DrawFloatInRange(-100.F, 100.F)));
            scaledImgPoints.push_back(cv::Point2f(DrawFloatInRange(-100.F, 100.F), DrawFloatInRange(-100.F, 100.F)));
        }

        std::vector<unsigned int> inlierIndices;
        cv::Mat1f translation;
        cv::Mat1f rotation;

        EXPECT_TRUE(VOCPP::DeltaPoseReconstruction::RecoverPoseRansac(scaledImgPoints, imgPoints, calMat, inlierIndices, translation, rotation));
        EXPECT_GE(inlierIndices.size(), 90);

        EXPECT_NEAR(rotTrue(0, 0), rotation(0, 0), 1e-2);
        EXPECT_NEAR(rotTrue(0, 1), rotation(0, 1), 1e-2);
        EXPECT_NEAR(rotTrue(0, 2), rotation(0, 2), 1e-2);
        EXPECT_NEAR(rotTrue(1, 0), rotation(1, 0), 1e-2);
        EXPECT_NEAR(rotTrue(1, 1), rotation(1, 1), 1e-2);
        EXPECT_NEAR(rotTrue(1, 2), rotation(1, 2), 1e-2);
        EXPECT_NEAR(rotTrue(2, 0), rotation(2, 0), 1e-2);
        EXPECT_NEAR(rotTrue(2, 1), rotation(2, 1), 1e-2);
        EXPECT_NEAR(rotTrue(2, 2), rotation(2, 2), 1e-2);

        // Translation is only known up to a global scale
        translationTrue = translationTrue / cv::norm(translationTrue);
        if (translationTrue(0, 0) / translation(0, 0) < 0.)
        {
            translation = -translation;
        }

        EXPECT_NEAR(translationTrue(0, 0), translation(0, 0), 1e-2);
        EXPECT_NEAR(translationTrue(1, 0), translation(1, 0), 1e-2);
        EXPECT_NEAR(translationTrue(2, 0), translation(2, 0), 1e-2);
    }
}
