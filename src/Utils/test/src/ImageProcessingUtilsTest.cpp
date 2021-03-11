/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <Vocpp_Utils/NumericalUtilities.h>
#include <Vocpp_Utils/FrameRotations.h>
#include <Vocpp_Utils/ConversionUtils.h>

#include <gtest/gtest.h>

using namespace VOCPP::Utils;

TEST(ExtractLocalMaximaTest, BlockMatrix)
{
    // Construct frame filled with ones
    cv::Mat1f ones = GetWindowKernel(5);

    std::vector<LocalMaximum> max;
    ExtractLocalMaxima(ones, 2U, max);
    
    // We expect 5 * 5 maxima since all values are equal
    EXPECT_EQ(max.size(), 25);

    // All values should be equal to 1.0
    for (auto val : max)
    {
        EXPECT_FLOAT_EQ(val.value, 1.0);
    }
}

TEST(ExtractLocalMaximaTest, OneMax)
{
    // Construct image with one center maxima
    cv::Mat1f ones = cv::Mat1f::zeros(5, 5);
    ones(2, 2) = 1.0;

    std::vector<LocalMaximum> max;
    ExtractLocalMaxima(ones, 2U, max);

    // We expect one single maximum
    EXPECT_EQ(max.size(), 1);
    EXPECT_FLOAT_EQ(max[0].posX, 2.0);
    EXPECT_FLOAT_EQ(max[0].posY, 2.0);
    EXPECT_FLOAT_EQ(max[0].value, 1.0);
}

TEST(ExtractLocalMaximaTest, TestDistanceCheck)
{
    // Construct image with three close-by maxima
    cv::Mat1f ones = cv::Mat1f::zeros(5, 5);
    ones(2, 2) = 1.0;
    ones(2, 3) = 3.0;
    ones(3, 2) = 4.0;

    std::vector<LocalMaximum> max;
    ExtractLocalMaxima(ones, 0U, max);

    // Without any distance check we expect three maxima
    EXPECT_EQ(max.size(), 3);
    EXPECT_FLOAT_EQ(max[0].posX, 2.0);
    EXPECT_FLOAT_EQ(max[0].posY, 2.0);
    EXPECT_FLOAT_EQ(max[0].value, 1.0);
    EXPECT_FLOAT_EQ(max[2].posX, 2.0);
    EXPECT_FLOAT_EQ(max[2].posY, 3.0);
    EXPECT_FLOAT_EQ(max[2].value, 4.0);
    EXPECT_FLOAT_EQ(max[1].posX, 3.0);
    EXPECT_FLOAT_EQ(max[1].posY, 2.0);
    EXPECT_FLOAT_EQ(max[1].value, 3.0);

    max.clear();
    // Do it again, this time with a minimum distance
    ExtractLocalMaxima(ones, 1U, max);

    // Only one maximum survives
    EXPECT_EQ(max.size(), 1);
    EXPECT_FLOAT_EQ(max[0].posX, 2.0);
    EXPECT_FLOAT_EQ(max[0].posY, 3.0);
    EXPECT_FLOAT_EQ(max[0].value, 4.0);
}

// Test essential matrix decomposition with translation only
TEST(DecomposeEssentialMatrixTest, PureTranslationTest)
{
    // Construct set of 3D points in world coordinate system
    std::vector<cv::Point3f> realWorldPoints;
    const float minDist = 150.0;
    for (int it = 0; it < 2; it++)
    {
        realWorldPoints.push_back(cv::Point3f(DrawFloatInRange(-minDist, minDist), DrawFloatInRange(-minDist, minDist), DrawFloatInRange(50.0, 150.0)));
    }

    // Kalibration matrix with shifted image center
    cv::Mat1f calMat = cv::Mat1f::zeros(3, 3);
    calMat(0, 0) = 5.0;
    calMat(1, 1) = 5.0;
    calMat(2, 2) = 1.0;
    calMat(0, 2) = 300;
    calMat(1, 2) = 300;

    for (auto coord : realWorldPoints)
    {
        // Projection matrix for left image, the translation is measured in the system of the left frame
        cv::Mat1f translationLeft = (cv::Mat1f(3, 1) << DrawFloatInRange(-1.0, 1.0), DrawFloatInRange(-1.0, 1.0), DrawFloatInRange(-5.0, 5.0));
        ImageProjectionMatrix projMatLeft(cv::Mat1f::eye(3, 3), translationLeft, calMat);

        // The right ones uses a trivial projection matrix
        ImageProjectionMatrix projMatRight(cv::Mat1f::eye(3, 3), cv::Mat1f::zeros(3, 1), calMat);

        // Get projected points in both camera frames
        std::vector<cv::Point2f> imgPointLeft;
        std::vector<cv::Point2f> imgPointRight;
        imgPointLeft.push_back(projMatLeft.Apply(coord));
        imgPointRight.push_back(projMatRight.Apply(coord));

        std::vector<unsigned int> inlierIndices;
        inlierIndices.push_back(0);

        // Compute true essential matrix (which is only given by the crossproduct matrix of the translation
        cv::Mat1f translatCross;
        GetCrossProductMatrix(translationLeft, translatCross);
        cv::Mat1f essentialMat = translatCross;

        // Decompose the essential matrix
        cv::Mat1f decompTranslat;
        cv::Mat1f decompRotMat;
        std::vector<cv::Point3f> triangulatedPoint;
        DecomposeEssentialMatrix(essentialMat, calMat, imgPointLeft, imgPointRight, inlierIndices, decompTranslat, decompRotMat, triangulatedPoint);
        // The true translation and the extracted one may differ by a global scale
        decompTranslat = decompTranslat * (translationLeft(0, 0) / decompTranslat(0, 0));

        // And check both extracted translation and rotation
        cv::Mat1f eye = cv::Mat1f::eye(3, 3);
        for (int rowIt = 0; rowIt < 3; rowIt++)
        {
            EXPECT_NEAR(decompTranslat.at<float>(rowIt, 0), translationLeft(rowIt, 0), 1e-2);
            for (int colIt = 0; colIt < 3; colIt++)
            {
                EXPECT_NEAR(decompRotMat.at<float>(rowIt, colIt), eye(rowIt, colIt), 1e-2);
            }
        }

        // Check triangulated point, may differ by a global scale
        EXPECT_NEAR(triangulatedPoint[0].y * coord.x / triangulatedPoint[0].x, coord.y, 0.1);
        EXPECT_NEAR(triangulatedPoint[0].z * coord.x / triangulatedPoint[0].x, coord.z, 0.1);
    }
  
}

// Test essential matrix decomposition with translation and rotation
TEST(DecomposeEssentialMatrixTest, TranslationAndRotationTest)
{
    // Construct set of 3D points in world coordinate system
    std::vector<cv::Point3f> realWorldPoints;
    const float minDist = 150.0;
    for (int it = 0; it < 150; it++)
    {
        realWorldPoints.push_back(cv::Point3f(DrawFloatInRange(-minDist, minDist), DrawFloatInRange(-minDist, minDist), DrawFloatInRange(50.0, 150.0)));
    }

    // Kalibration matrix with shifted image center
    cv::Mat1f calMat = cv::Mat1f::zeros(3, 3);
    calMat(0, 0) = 5.0;
    calMat(1, 1) = 5.0;
    calMat(2, 2) = 1.0;
    calMat(0, 2) = 300;
    calMat(1, 2) = 300;

    for (auto coord : realWorldPoints)
    {
        // Projection matrix for left image, the translation is measured in the system of the left frame
        cv::Mat1f translationLeft = (cv::Mat1f(3, 1) << DrawFloatInRange(-10.0F, 10.0F), DrawFloatInRange(-10.0F, 10.0F), DrawFloatInRange(-5.0F, 5.0F));
        cv::Mat1f rotMat = GetFrameRotationX(DrawFloatInRange(-0.2F, 0.2F)) * GetFrameRotationY(DrawFloatInRange(-0.2F, 0.2F)) * GetFrameRotationZ(DrawFloatInRange(-0.2F, 0.2F));
        ImageProjectionMatrix projMatLeft(rotMat, translationLeft, calMat);

        // The right ones uses a trivial projection matrix
        ImageProjectionMatrix projMatRight(cv::Mat1f::eye(3, 3), cv::Mat1f::zeros(3, 1), calMat);

        // Get projected points in both camera frames
        std::vector<cv::Point2f> imgPointLeft;
        std::vector<cv::Point2f> imgPointRight;
        imgPointLeft.push_back(projMatLeft.Apply(coord));
        imgPointRight.push_back(projMatRight.Apply(coord));

        std::vector<unsigned int> inlierIndices;
        inlierIndices.push_back(0);

        // Compute true essential matrix (which is only given by the crossproduct matrix of the translation
        cv::Mat1f translatCross;
        GetCrossProductMatrix(translationLeft, translatCross);
        cv::Mat1f essentialMat = translatCross * rotMat;

        // Decompose the essential matrix
        cv::Mat1f decompTranslat;
        cv::Mat1f decompRotMat;
        std::vector<cv::Point3f> triangulatedPoint;
        DecomposeEssentialMatrix(essentialMat, calMat, imgPointLeft, imgPointRight, inlierIndices, decompTranslat, decompRotMat, triangulatedPoint);
        // The true translation and the extracted one may differ by a global scale
        decompTranslat = decompTranslat * (translationLeft(0, 0) / decompTranslat(0, 0));

        // And check both extracted translation and rotation
        cv::Mat1f eye = cv::Mat1f::eye(3, 3);
        for (int rowIt = 0; rowIt < 3; rowIt++)
        {
            EXPECT_NEAR(decompTranslat(rowIt, 0), translationLeft(rowIt, 0), std::abs(1e-2 * translationLeft(rowIt, 0)));
            for (int colIt = 0; colIt < 3; colIt++)
            {
                EXPECT_NEAR(decompRotMat(rowIt, colIt), rotMat(rowIt, colIt), 1e-4);
            }
        }
    }

}

TEST(PointTriangulationLinearTest, TwoCamerasRotationAndTranslation)
{
    // Construct set of 3D points in world coordinate system
    std::vector<cv::Point3f> realWorldPoints;
    const float minDist = 150.0;
    for (int it = 0; it < 100; it++)
    {
        realWorldPoints.push_back(cv::Point3f(DrawFloatInRange(-minDist, minDist), DrawFloatInRange(-minDist, minDist), DrawFloatInRange(50.0, 150.0)));
    }

    // Kalibration matrix with shifted image center
    cv::Mat1f calMat = cv::Mat1f::zeros(3, 3);
    calMat(0, 0) = 5.0;
    calMat(1, 1) = 5.0;
    calMat(2, 2) = 1.0;
    calMat(0, 2) = 300;
    calMat(1, 2) = 300;

    std::vector<cv::Point2f> imgPointsLeft;
    std::vector<cv::Point2f> imgPointsRight;

    // Projection matrix for left image
    cv::Mat1f rotMatLeft = GetFrameRotationX(DrawFloatInRange(-0.5, 0.5)) * GetFrameRotationY(DrawFloatInRange(-0.5, 0.5)) * GetFrameRotationZ(DrawFloatInRange(-0.5, 0.5));
    cv::Mat1f translationLeft = (cv::Mat1f(3, 1) << DrawFloatInRange(-1.0, 5.0), DrawFloatInRange(-1.0, 5.0), DrawFloatInRange(-5.0, 5.0));
    ImageProjectionMatrix projMatLeft(rotMatLeft, translationLeft, calMat);
    
    // Projection matrix for right image
    cv::Mat1f rotMatRight = GetFrameRotationX(DrawFloatInRange(-0.5, 0.5)) * GetFrameRotationY(DrawFloatInRange(-0.5, 0.5)) * GetFrameRotationZ(DrawFloatInRange(-0.5, 0.5));
    cv::Mat1f translationRight = (cv::Mat1f(3, 1) << DrawFloatInRange(-1.0, 5.0), DrawFloatInRange(-1.0, 5.0), DrawFloatInRange(-5.0, 5.0));
    ImageProjectionMatrix projMatRight(rotMatRight, translationRight, calMat);


    // Get image coordinates of real world points
    for (auto coord : realWorldPoints)
    {
        imgPointsLeft.push_back(projMatLeft.Apply(coord));
        imgPointsRight.push_back(projMatRight.Apply(coord));
    }

    // Triangulate points and check
    for (int it = 0; it < static_cast<int>(imgPointsLeft.size()); it++)
    {
        cv::Point3f triangPoint;
        PointTriangulationLinear(projMatLeft, projMatRight, imgPointsLeft[it], imgPointsRight[it], triangPoint);
        EXPECT_NEAR(triangPoint.x, realWorldPoints[it].x, 1e-1);
        EXPECT_NEAR(triangPoint.y, realWorldPoints[it].y, 1e-1);
        EXPECT_NEAR(triangPoint.z, realWorldPoints[it].z, 1e-1);
    }
}
