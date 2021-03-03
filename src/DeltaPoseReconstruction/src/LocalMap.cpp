/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_DeltaPoseReconstruction/LocalMap.h>
#include <algorithm>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

/////////////////////////////////
//          LANDMARK           //
/////////////////////////////////

void Landmark::UpdateLandmark(const unsigned int& in_currentFrameId, const unsigned int& in_currentFeatureId,
    const unsigned int& in_lastFrameId, const unsigned int& in_lastFeatureId, const LandmarkPosition& in_position)
{
    // Only add occurence in current frame (should be present in last one already)
    m_frameVsFeatureId.insert(std::pair<int, int>(in_currentFrameId, in_currentFeatureId));
    // Add triangulated position
    FramePairKey keyPair{ in_currentFrameId, in_lastFrameId };
    m_framePairVsPosition.insert(std::pair<FramePairKey, LandmarkPosition>(keyPair, in_position));
    m_lastSeenFrameId = in_currentFrameId;
}

bool Landmark::IsPresentInFrameWithFeatureId(const unsigned int& in_frameId, const unsigned int& in_featureId) const
{
    bool ret = m_frameVsFeatureId.count(in_frameId) > 0 ? true : false;

    if (ret)
    {
        ret = m_frameVsFeatureId.at(in_frameId) == in_featureId ? true : false;
    }

    return ret;
}

bool Landmark::IsPresentInFrame(const unsigned int& in_frameId) const
{
    return m_frameVsFeatureId.count(in_frameId) > 0 ? true : false;
}


bool Landmark::GetFramePairPosition(const FramePairKey& in_pairKey, LandmarkPosition& out_position) const
{
    std::map<FramePairKey,LandmarkPosition>::const_iterator it = m_framePairVsPosition.find(in_pairKey);
    bool ret = it != m_framePairVsPosition.end();
    if (ret)
    {
        out_position = it->second;
    }

    return ret;
}

bool Landmark::HasBeenScaledForFramePair(const FramePairKey& in_key) const
{
    std::map<FramePairKey, bool>::const_iterator it =m_framePairVsScaled.find(in_key);
    bool ret = it != m_framePairVsScaled.end();
    if (ret)
    {
        ret = it->second;
    }

    return ret;
}

bool Landmark::RescalePosition(const FramePairKey& in_pairKey, const float& in_scale)
{
    std::map<FramePairKey, LandmarkPosition>::iterator it = m_framePairVsPosition.find(in_pairKey);
    bool ret = it != m_framePairVsPosition.end();
    if (ret)
    {
        it->second.x *= in_scale;
        it->second.y *= in_scale;
        it->second.z *= in_scale;

        m_framePairVsScaled.insert(std::pair<FramePairKey, bool>(in_pairKey, true));
    }

    return ret;
}



/////////////////////////////////
//          LOCAL MAP          //
/////////////////////////////////

LocalMap::~LocalMap()
{
    m_landmarks.clear();
}

void LocalMap::InsertLandmarks(const std::vector<LandmarkPosition>& in_positions, const std::vector<FeatureHandling::BinaryDescriptionMatch>& in_matches, const unsigned int& in_currentFrameId)
{
    for (unsigned int it = 0U; it < in_positions.size(); it++)
    {
        // Check whether this landmark has been observed before
        bool found = false;
        // We assume here that first frame == current frame and second frame == last frame
        // TODO: Refactor the match interface!
        assert(in_currentFrameId == in_matches[it].GetFirstFeature().frameId);
        for (auto& mark : m_landmarks)
        {
            if (mark.IsPresentInFrameWithFeatureId(in_matches[it].GetSecondFeature().frameId, in_matches[it].GetSecondFeature().id))
            {
                mark.UpdateLandmark(in_currentFrameId, in_matches[it].GetFirstFeature().id, in_matches[it].GetSecondFeature().frameId, in_matches[it].GetSecondFeature().id, in_positions[it]);
                found = true;
            }
        }

        // If Landmark has not been found --> create new one
        if (!found)
        {
            m_landmarks.push_back(Landmark(in_currentFrameId, in_matches[it].GetFirstFeature().id, in_matches[it].GetSecondFeature().frameId, in_matches[it].GetSecondFeature().id, in_positions[it]));
        }
    }

    RemoveUntrackedLandmarks(in_currentFrameId);

    ComputeRelativeScale(in_currentFrameId);
    m_lastFrameId = in_currentFrameId;
}

void LocalMap::RemoveUntrackedLandmarks(const unsigned int& in_currentFrameId)
{
    auto newEnd = std::remove_if(m_landmarks.begin(), m_landmarks.end(), [&in_currentFrameId](const Landmark& i) {return i.GetLastSeenFrameId() < in_currentFrameId; });
    m_landmarks.erase(newEnd, m_landmarks.end());
}

bool LocalMap::GetLastRelativeScale(const unsigned int& in_lastFrameId, const unsigned int& in_currentFrameId, float& out_scale) const
{
    bool ret = false;

    if (m_lastFrameId == in_currentFrameId && (m_lastFrameId - 1U) == in_lastFrameId && m_validTrackedLandmarks >= m_minNumberOfTrackedLandmarks)
    {
        ret = true;
        out_scale = m_lastRelativeScale;
    }

    return ret;
}


void LocalMap::ComputeRelativeScale(const unsigned int& in_currentFrameId)
{
    std::vector<LandmarkPosition> currentPosVec;
    std::vector<LandmarkPosition> lastPosVec;
    std::vector<int> validLandmarkIndices;

    for (unsigned int it=0U; it < m_landmarks.size(); it++)
    {
        // Check whether the landmark can be tracked three frames into the past (including the current one)
        Landmark::FramePairKey keyCurrent{ in_currentFrameId, in_currentFrameId - 1U};
        LandmarkPosition posCurrent;
        Landmark::FramePairKey keyLast{ in_currentFrameId - 1U, in_currentFrameId - 2U};
        LandmarkPosition posLast;
        bool present = m_landmarks[it].GetFramePairPosition(keyLast, posLast) && m_landmarks[it].GetFramePairPosition(keyCurrent, posCurrent);
        bool lastScaled = m_landmarks[it].HasBeenScaledForFramePair(keyLast);
        if (present && lastScaled)
        {
            currentPosVec.push_back(posCurrent);
            lastPosVec.push_back(posLast);
            validLandmarkIndices.push_back(it);
        }
    }

    // Update number of valid tracked landmarks
    m_validTrackedLandmarks = static_cast<unsigned int>(currentPosVec.size());

    if (currentPosVec.size() >= 2U)
    {
        std::vector<float> scales;
        for (unsigned int i = 0U; i < currentPosVec.size() - 1U; i++)
        {
            unsigned int j = i + 1U;
            const float lastFramePairDist = std::sqrt(std::pow(lastPosVec[i].x - lastPosVec[j].x, 2)
                    + std::pow(lastPosVec[i].y - lastPosVec[j].y, 2)
                    + std::pow(lastPosVec[i].z - lastPosVec[j].z, 2));
            const float currentFramePairDist = std::sqrt(std::pow(currentPosVec[i].x - currentPosVec[j].x, 2)
                    + std::pow(currentPosVec[i].y - currentPosVec[j].y, 2)
                    + std::pow(currentPosVec[i].z - currentPosVec[j].z, 2));

            // Compute relative scale
            scales.push_back(lastFramePairDist / currentFramePairDist);
        }

        std::nth_element(scales.begin(), scales.begin() + scales.size() / 2, scales.end());
        // Update current scale
        m_lastRelativeScale = scales[scales.size() / 2U];
        std::cout << "Tracked Landmarks: " << m_validTrackedLandmarks << std::endl;

        for (auto it : validLandmarkIndices)
        {
            // Check whether the landmark can be tracked three frames into the past (including the current one)
            Landmark::FramePairKey keyCurrent{ in_currentFrameId, in_currentFrameId - 1U };
            LandmarkPosition posCurrent;
            m_landmarks[it].GetFramePairPosition(keyCurrent, posCurrent);
            (void)m_landmarks[it].RescalePosition(keyCurrent, m_lastRelativeScale);
            m_landmarks[it].GetFramePairPosition(keyCurrent, posCurrent);
        }
    }
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP