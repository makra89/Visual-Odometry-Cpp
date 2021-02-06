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
using VOCPP::Utils::ImageProjectionMatrix;
using VOCPP::Utils::GetCrossProductMatrix;

/* We restrict the fundamental matrix to translation */
TEST(PureTranslationModel, Translation)
{
    for (int testIt = 0; testIt < 100; testIt++)
    {
        VOCPP::DeltaPoseReconstruction::PureTranslationModel model;

        std::vector<cv::Point3f> realWorldPoints;
        std::vector<cv::Point2f> imgPoints;
        std::vector<cv::Point2f> scaledImgPoints;

        const double centerX = 400.F;
        const double centerY = 600.F;

        // Construct set of 3D points in world coordinate system
        const float maxDist = 3.0;
        // Use minimum number of correspondences
        for (int it = 0; it < model.GetNumCorrespondences(); it++)
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

        // Projection matrix with translation only
        cv::Mat1f translationTrue = (cv::Mat1f(3, 1) << DrawFloatInRange(-5.F, 5.F), DrawFloatInRange(-5.F, 5.F), DrawFloatInRange(-2.0F, 2.0F));
        cv::Mat1f rotTrue = cv::Mat1f::eye(3, 3);
        ImageProjectionMatrix projMat(rotTrue, translationTrue, calMat);

        for (auto coord : realWorldPoints)
        {
            scaledImgPoints.push_back(projMat.Apply(coord));
        }

        // Get fundamental matrix
        std::vector<cv::Mat1f> solutionVec;
        model.Compute(scaledImgPoints, imgPoints, solutionVec);
        ASSERT_TRUE(solutionVec.size() == 1);

        // Extract rotation and translation
        cv::Mat1f translation;
        cv::Mat1f rot;
        model.DecomposeSolution(solutionVec[0], calMat, scaledImgPoints, imgPoints, translation, rot);

        EXPECT_DOUBLE_EQ(rotTrue(0, 0), rot(0, 0));
        EXPECT_DOUBLE_EQ(rotTrue(0, 1), rot(0, 1));
        EXPECT_DOUBLE_EQ(rotTrue(0, 2), rot(0, 2));
        EXPECT_DOUBLE_EQ(rotTrue(1, 0), rot(1, 0));
        EXPECT_DOUBLE_EQ(rotTrue(1, 1), rot(1, 1));
        EXPECT_DOUBLE_EQ(rotTrue(1, 2), rot(1, 2));
        EXPECT_DOUBLE_EQ(rotTrue(2, 0), rot(2, 0));
        EXPECT_DOUBLE_EQ(rotTrue(2, 1), rot(2, 1));
        EXPECT_DOUBLE_EQ(rotTrue(2, 2), rot(2, 2));

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
