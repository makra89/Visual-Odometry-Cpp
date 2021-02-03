/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_DeltaPoseReconstruction/LocalMap.h>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

using VOCPP::DeltaPoseReconstruction::LocalMap;
using VOCPP::DeltaPoseReconstruction::Landmark;
using VOCPP::DeltaPoseReconstruction::LandmarkPosition;
using VOCPP::FeatureHandling::Feature;
using VOCPP::FeatureHandling::BinaryFeatureDescription;
using VOCPP::FeatureHandling::BinaryDescriptionMatch;

// Check that it is possible to create new Landmarks for new frames and/or new keypoint ids
TEST(LocalMapTest, CreateNewLandmarks)
{
    // Create map and check that its empty
    LocalMap map(5U);
    EXPECT_TRUE(map.GetLandmarks().size() == 0);
    
    std::vector<LandmarkPosition> landmarks;
    std::vector<BinaryDescriptionMatch> matches;
    LandmarkPosition dummyLandmark = { 1.0, 2.0, 3.0 };
    unsigned int currentFrameId = 1U;
    unsigned int lastFrameId = 0U;
    
    // Create a landmark with current frame ID = 1 and feature Id = 5
    // and last frame ID = 0 and feature Id = 7
    std::vector<uint8_t> dummyDesc;
    BinaryFeatureDescription desc1(Feature{ 7U, 0U }, dummyDesc);
    BinaryFeatureDescription desc2(Feature{ 5U, 1U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch match(desc2, desc1, 0.F);
    landmarks.push_back(dummyLandmark);
    matches.push_back(match);

    // Create another one with different feature Ids
    BinaryFeatureDescription desc3(Feature{ 11U, 0U }, dummyDesc);
    BinaryFeatureDescription desc4(Feature{ 25U, 1U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch newMatch(desc4, desc3, 0.F);
    landmarks.push_back(dummyLandmark);
    matches.push_back(newMatch);
    
    // Insert landmarks into map
    map.InsertLandmarks(landmarks, matches, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 2);

    // Check for current and last frame
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(currentFrameId, 5U));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(lastFrameId, 7U));
    EXPECT_TRUE(map.GetLandmarks()[1].IsPresentInFrameWithFeatureId(currentFrameId, 25U));
    EXPECT_TRUE(map.GetLandmarks()[1].IsPresentInFrameWithFeatureId(lastFrameId, 11U));
    // Check also other interface
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(currentFrameId));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrame(lastFrameId));
    EXPECT_TRUE(map.GetLandmarks()[1].IsPresentInFrame(currentFrameId));
    EXPECT_TRUE(map.GetLandmarks()[1].IsPresentInFrame(lastFrameId));
}

// Check that it is possible to update an existing landmark if it has been seen in 
// a more up-to-date frame
TEST(LocalMapTest, UpdateExistingLandmarks)
{
    // Create map and check that its empty
    LocalMap map(10U);
    EXPECT_TRUE(map.GetLandmarks().size() == 0);

    std::vector<LandmarkPosition> landmarks;
    std::vector<BinaryDescriptionMatch> matches;
    LandmarkPosition dummyLandmark = { 1.0, 2.0, 3.0 };
    unsigned int currentFrameId = 1U;
    unsigned int lastFrameId = 0U;

    // Create a landmark with current frame ID = 1 and feature Id = 5
    // and last frame ID = 0 and feature Id = 7
    std::vector<uint8_t> dummyDesc;
    BinaryFeatureDescription desc1(Feature{ 7U, 0U }, dummyDesc);
    BinaryFeatureDescription desc2(Feature{ 5U, 1U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch match(desc2, desc1, 0.F);
    landmarks.push_back(dummyLandmark);
    matches.push_back(match);
    // And insert landmark into map
    map.InsertLandmarks(landmarks, matches, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 1);

    // Check for current and last frame
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(currentFrameId, 5U));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(lastFrameId, 7U));

    // Create the same landmark tracked to a new frame
    landmarks.clear();
    matches.clear();
    currentFrameId = 2U;
    lastFrameId = 1U;
    BinaryFeatureDescription desc3(Feature{ 11U, 2U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch newMatch(desc3, desc2, 0.F);
    landmarks.push_back(dummyLandmark);
    matches.push_back(newMatch);
    map.InsertLandmarks(landmarks, matches, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 1);

    // Check for all frames so far
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(0U, 7U));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(1U, 5U));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(2U, 11U));

    // And to another one
    landmarks.clear();
    matches.clear();
    currentFrameId = 3U;
    lastFrameId = 2U;
    BinaryFeatureDescription desc4(Feature{ 25U, 3U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch evenNewerMatch(desc4, desc3, 0.F);
    landmarks.push_back(dummyLandmark);
    matches.push_back(evenNewerMatch);
    map.InsertLandmarks(landmarks, matches, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 1);

    // Check for current and last frame
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(0U, 7U));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(1U, 5U));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(2U, 11U));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(3U, 25U));
}

// Check that it is possible to create new Landmarks for new frames and/or new keypoint ids
TEST(LocalMapTest, RemoveUntrackedLandmarks)
{
    // Create map and check that its empty
    LocalMap map(5U);
    EXPECT_TRUE(map.GetLandmarks().size() == 0);

    std::vector<LandmarkPosition> landmarks;
    std::vector<BinaryDescriptionMatch> matches;
    LandmarkPosition dummyLandmark = { 1.0, 2.0, 3.0 };
    unsigned int currentFrameId = 1U;
    unsigned int lastFrameId = 0U;

    // Create a landmark with current frame ID = 1 and feature Id = 5
    // and last frame ID = 0 and feature Id = 7
    std::vector<uint8_t> dummyDesc;
    BinaryFeatureDescription desc1(Feature{ 7U, 0U }, dummyDesc);
    BinaryFeatureDescription desc2(Feature{ 5U, 1U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch match(desc2, desc1, 0.F);
    landmarks.push_back(dummyLandmark);
    matches.push_back(match);

    // Create another one with different feature Ids
    BinaryFeatureDescription desc3(Feature{ 11U, 0U }, dummyDesc);
    BinaryFeatureDescription desc4(Feature{ 25U, 1U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch newMatch(desc4, desc3, 0.F);
    landmarks.push_back(dummyLandmark);
    matches.push_back(newMatch);
    map.InsertLandmarks(landmarks, matches, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 2);

    // Check for current and last frame
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(currentFrameId, 5U));
    EXPECT_TRUE(map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(lastFrameId, 7U));
    EXPECT_TRUE(map.GetLandmarks()[1].IsPresentInFrameWithFeatureId(currentFrameId, 25U));
    EXPECT_TRUE(map.GetLandmarks()[1].IsPresentInFrameWithFeatureId(lastFrameId, 11U));

    // Now go on to another frame and only track the second landmark
    matches.clear();
    landmarks.clear();
    currentFrameId = 2U;
    lastFrameId = 1U;
    BinaryFeatureDescription desc5(Feature{ 31U, 2U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch evenNewerMatch(desc5, desc4, 0.F);
    landmarks.push_back(dummyLandmark);
    matches.push_back(evenNewerMatch);
    map.InsertLandmarks(landmarks, matches, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 2);

    // Now remove all untracked landmarks (should remove the first one)
    map.RemoveUntrackedLandmarks(currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 1);
    // Check that we really removed the correct landmark
    map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(lastFrameId, 25U);
    map.GetLandmarks()[0].IsPresentInFrameWithFeatureId(currentFrameId, 31U);
}

// Check that it is possible to create new Landmarks for new frames and/or new keypoint ids
TEST(LocalMapTest, CalculateRelativeScale)
{
    // Create map and check that its empty
    // By intent allow scale calculation with the minimum number possible
    LocalMap map(2U);
    EXPECT_TRUE(map.GetLandmarks().size() == 0);

    std::vector<LandmarkPosition> landmarks;
    std::vector<BinaryDescriptionMatch> matches;
    LandmarkPosition landmarkPos1 = { 1.0, 2.0, 3.0 };
    LandmarkPosition landmarkPos2 = { 1.5, 2.5, 3.5 };
    unsigned int currentFrameId = 1U;
    unsigned int lastFrameId = 0U;
    // Insert two landmarks
    std::vector<uint8_t> dummyDesc;
    BinaryFeatureDescription desc1(Feature{ 0U, 0U }, dummyDesc);
    BinaryFeatureDescription desc2(Feature{ 0U, 1U }, dummyDesc);
    BinaryFeatureDescription desc3(Feature{ 1U, 0U }, dummyDesc);
    BinaryFeatureDescription desc4(Feature{ 1U, 1U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch match1(desc2, desc1, 0.F);
    VOCPP::FeatureHandling::BinaryDescriptionMatch match2(desc4, desc3, 0.F);
    landmarks.push_back(landmarkPos1);
    landmarks.push_back(landmarkPos2);
    matches.push_back(match1);
    matches.push_back(match2);
    map.InsertLandmarks(landmarks, matches, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 2);

    // Now go on to another frame and track the landmarks with a new position (simulate scale factor of 2.0)
    matches.clear();
    landmarks.clear();
    LandmarkPosition newLandmarkPos1 = { 2.0, 4.0, 6.0 };
    LandmarkPosition newLandmarkPos2 = { 3.0, 5.0, 7.0 };
    currentFrameId = 2U;
    lastFrameId = 1U;
    BinaryFeatureDescription desc5(Feature{ 0U, 2U }, dummyDesc);
    BinaryFeatureDescription desc6(Feature{ 1U, 2U }, dummyDesc);
    VOCPP::FeatureHandling::BinaryDescriptionMatch newMatch1(desc5, desc2, 0.F);
    VOCPP::FeatureHandling::BinaryDescriptionMatch newMatch2(desc6, desc4, 0.F);
    landmarks.push_back(newLandmarkPos1);
    landmarks.push_back(newLandmarkPos2);
    matches.push_back(newMatch1);
    matches.push_back(newMatch2);
    map.InsertLandmarks(landmarks, matches, currentFrameId);
    ASSERT_TRUE(map.GetLandmarks().size() == 2);

    float relativeScale = 0.0;
    EXPECT_TRUE(map.GetLastRelativeScale(1U /*last frame*/, 2U /*current frame*/, relativeScale));
    EXPECT_FLOAT_EQ(relativeScale, 0.5);

    // Check that the positions in the last frame pair have been rescaled
    LandmarkPosition position;
    Landmark::FramePairKey key{ currentFrameId, lastFrameId };
    EXPECT_TRUE(map.GetLandmarks()[0].GetFramePairPosition(key, position));
    EXPECT_DOUBLE_EQ(position.x, landmarkPos1.x);
    EXPECT_DOUBLE_EQ(position.y, landmarkPos1.y);
    EXPECT_DOUBLE_EQ(position.z, landmarkPos1.z);

    EXPECT_TRUE(map.GetLandmarks()[1].GetFramePairPosition(key, position));
    EXPECT_DOUBLE_EQ(position.x, landmarkPos2.x);
    EXPECT_DOUBLE_EQ(position.y, landmarkPos2.y);
    EXPECT_DOUBLE_EQ(position.z, landmarkPos2.z);
}

