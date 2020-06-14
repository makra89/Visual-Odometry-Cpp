/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_DeltaPoseReconstruction/LocalMap.h>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

using namespace VOCPP::DeltaPoseReconstruction;
using VOCPP::FeatureHandling::Feature;
using VOCPP::FeatureHandling::BinaryFeatureDescription;
using VOCPP::FeatureHandling::BinaryDescriptionMatch;

// Check that it is possible to create new Landmarks for new frames and/or new keypoint ids
TEST(LocalMapTest, CreateNewLandmarks)
{
    // Create map and check that its empty
    LocalMap map(5U);
    EXPECT_TRUE(map.GetLandmarks().size() == 0);
    
    LandmarkPosition dummyLandmark = { 1.0, 2.0, 3.0 };
    unsigned int currentFrameId = 1U;
    unsigned int lastFrameId = 0U;
    // Insert a landmark with current frame ID = 1 and feature Id = 5
    // and last frame ID = 0 and feature Id = 7
    std::vector<bool> dummyDesc;
    BinaryFeatureDescription desc1(Feature{ 7U, 0U }, dummyDesc);
    BinaryFeatureDescription desc2(Feature{ 5U, 1U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch match(desc2, desc1, 0U);
    map.InsertLandmark(dummyLandmark, match, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 1);

    // Check for current and last frame
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(currentFrameId, 5));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(lastFrameId, 7));

    // Insert another one with different feature Ids
    BinaryFeatureDescription desc3(Feature{ 11U, 0U }, dummyDesc);
    BinaryFeatureDescription desc4(Feature{ 25U, 1U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch newMatch(desc4, desc3, 0U);
    map.InsertLandmark(dummyLandmark, newMatch, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 2);

    // Check for current and last frame
    EXPECT_TRUE(map.GetLandmarks()[1].IsPresentInFrame(currentFrameId, 25));
    EXPECT_TRUE(map.GetLandmarks()[1].IsPresentInFrame(lastFrameId, 11));
}

// Check that it is possible to update an existing landmark if it has been seen in 
// a more up-to-date frame
TEST(LocalMapTest, UpdateExistingLandmarks)
{
    // Create map and check that its empty
   /* LocalMap map;
    EXPECT_TRUE(map.GetLandmarks().size() == 0);

    LandmarkPosition dummyLandmark = { 1.0, 2.0, 3.0 };
    int currentFrameId = 1;
    int lastFrameId = 0;
    // Insert a landmark with current frame ID = 1 and keypoint Id = 5
    // and last frame ID = 0 and keypoint Id = 7
    cv::DMatch match(5, 7, lastFrameId, 0);
    map.InsertLandmark(dummyLandmark, match, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 1);

    // Check for current and last frame
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(currentFrameId, 5));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(lastFrameId, 7));

    // Insert the same landmark tracked to a new frame
    currentFrameId = 2;
    lastFrameId = 1;
    cv::DMatch newMatch(9, 5, lastFrameId, 0);
    map.InsertLandmark(dummyLandmark, newMatch, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 1);

    // Check for current and last frame
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(0, 7));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(1, 5));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(2, 9));

    // And to another one
    currentFrameId = 3;
    lastFrameId = 2;
    cv::DMatch evenNewerMatch(12, 9, lastFrameId, 0);
    map.InsertLandmark(dummyLandmark, evenNewerMatch, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 1);

    // Check for current and last frame
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(0, 7));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(1, 5));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(2, 9));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(3, 12));*/
}
