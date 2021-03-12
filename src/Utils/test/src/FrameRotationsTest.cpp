/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/FrameRotations.h>
#include <gtest/gtest.h>

TEST(ExtractRollPitchYawTest, ExtractAngles)
{
    cv::Mat1d rotX = VOCPP::Utils::GetFrameRotationX(0.51F);
    cv::Mat1d rotY = VOCPP::Utils::GetFrameRotationY(-0.33F);
    cv::Mat1d rotZ = VOCPP::Utils::GetFrameRotationZ(0.85F);
    cv::Mat1d combinedRot = rotX * rotY * rotZ;

    cv::Vec3f eulerAngles = VOCPP::Utils::ExtractRollPitchYaw(combinedRot);

    EXPECT_NEAR(eulerAngles[0], 0.51F, 1e-6);
    EXPECT_NEAR(eulerAngles[1], -0.33F, 1e-6);
    EXPECT_NEAR(eulerAngles[2], 0.85F, 1e-6);
}
