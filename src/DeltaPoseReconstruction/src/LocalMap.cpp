/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_DeltaPoseReconstruction/LocalMap.h>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

LocalMap::~LocalMap()
{
    m_landmarks.clear();
}

void Landmark::UpdateLandmark(const unsigned int& in_currentFrameId, const unsigned int& in_currentFeatureId,
    const unsigned int& in_lastFrameId, const unsigned int& in_lastFeatureId, const LandmarkPosition& in_position)
{
    // Only add occurence in current frame (should be present in last one already)
    m_frameVsFeatureId.insert(std::pair<int, int>(in_currentFrameId, in_currentFeatureId));
    // Add triangulated position
    FramePairKey keyPair{ in_currentFrameId, in_lastFrameId };
    m_framePairVsPosition.insert(std::pair<FramePairKey, LandmarkPosition>(keyPair, in_position));
    m_lastSeenFrameId = in_currentFrameId;

    std::cout << "Found keypoint" << std::endl;
    for (auto bla : m_frameVsFeatureId)
    {
        std::cout << bla.first << " " << bla.second << std::endl;
    }
}

bool Landmark::IsPresentInFrame(const unsigned int& in_frameId, const unsigned int& in_featureId) const
{
    bool ret = m_frameVsFeatureId.count(in_frameId) > 0 ? true : false;

    if (ret)
    {
        ret = m_frameVsFeatureId.at(in_frameId) == in_featureId ? true : false;
    }

    return ret;
}

void LocalMap::InsertLandmark(const LandmarkPosition& in_position, const FeatureHandling::BinaryDescriptionMatch& in_match, const unsigned int& in_currentFrameId)
{
    // Check whether this landmark has been observed before
    bool found = false;
    // "Tracked" is defined as a landmark that is at least visible in three frames
    unsigned int trackedLandmarks = 0U;
    // We assume here that first frame == current frame and second frame == last frame
    // TODO: Refactor the match interface!
    assert(in_currentFrameId == in_match.GetFirstFeature().frameId);

    for (auto& mark : m_landmarks)
    {
        if (mark.IsPresentInFrame(in_match.GetSecondFeature().frameId, in_match.GetSecondFeature().id))
        {
            mark.UpdateLandmark(in_currentFrameId, in_match.GetFirstFeature().id, in_match.GetSecondFeature().frameId, in_match.GetSecondFeature().id, in_position);
            trackedLandmarks++;
            found = true;
        }
    }

    // If Landmark has not been found --> create new one
    if (!found)
    {
        m_landmarks.push_back(Landmark(in_currentFrameId, in_match.GetFirstFeature().id, in_match.GetSecondFeature().frameId, in_match.GetSecondFeature().id, in_position));
    }
}

} //namespace DeltaPoseReconstruction
} //namespace VOCPP