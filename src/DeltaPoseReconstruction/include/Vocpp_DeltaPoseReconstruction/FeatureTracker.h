/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_FEATURE_TRACKER_H
#define VOCPP_FEATURE_TRACKER_H

#include <Vocpp_FeatureHandling/LshMatcher.h>

#include <map>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

class TrackedFeature
{
public:
    TrackedFeature(unsigned int in_currentFrameId, unsigned int in_currentFeatureId, const FeatureHandling::BinaryFeatureDescription& in_currentDesc) :
        m_frameIdVsFeatureId(),
        m_frameIdVsDescription()
    {
        Update(in_currentFrameId, in_currentFeatureId, in_currentDesc);
    }
    
    bool IsPresentInFrame(unsigned int in_frameId) const
    {
        return m_frameIdVsFeatureId.count(in_frameId) > 0 ? true : false;
    }

    bool IsPresentInFrameWithFeatureId(const unsigned int& in_frameId, const unsigned int& in_featureId) const
    {
        bool ret = m_frameIdVsFeatureId.count(in_frameId) > 0 ? true : false;

        if (ret)
        {
            ret = m_frameIdVsFeatureId.at(in_frameId) == in_featureId ? true : false;
        }

        return ret;
    }

    void Update(unsigned int in_currentFrameId, unsigned int in_currentFeatureId, const FeatureHandling::BinaryFeatureDescription& in_currentDesc)
    {
        // Only add occurence in current frame (should be present in last one already)
        m_frameIdVsFeatureId.insert(std::pair<int, int>(in_currentFrameId, in_currentFeatureId));
        m_frameIdVsDescription.insert(std::pair<int, FeatureHandling::BinaryFeatureDescription>(in_currentFrameId, in_currentDesc));
        m_lastSeenFrameId = in_currentFrameId;
    }

    unsigned int m_lastSeenFrameId;
    std::map<int, int> m_frameIdVsFeatureId;
    std::map<int, FeatureHandling::BinaryFeatureDescription> m_frameIdVsDescription;
};

struct TrackedFeatureSet
{
    std::vector<TrackedFeature> features;
    bool isKeySet;
};

class FeatureTracker
{
public:

    FeatureTracker(unsigned int in_minTrackedTwoFrames=100U, unsigned int in_minTrackedThreeFrames=10U)
        : m_matcher(), 
        m_currentKeyFeatureSet(), 
        m_lastKeyFrameId(s_invalidFrameId),
        m_KeyFrameIdBeforeLast(s_invalidFrameId),
        m_lastKeyFrameDescriptions(),
        m_minTrackedTwoFrames(in_minTrackedTwoFrames),
        m_minTrackedThreeFrames(in_minTrackedThreeFrames),
        m_trackingStatus(TrackingStatus::Lost)
    {
    }
   
    TrackedFeatureSet ProvideDescriptionSet(unsigned int in_currentFrameId, std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription>& in_currentDesc);

private:

    FeatureHandling::LshMatcher m_matcher;
    TrackedFeatureSet m_currentKeyFeatureSet;

    unsigned int m_lastKeyFrameId;
    unsigned int m_KeyFrameIdBeforeLast;
    std::vector<FeatureHandling::BinaryFeatureDescription> m_lastKeyFrameDescriptions;
    const unsigned int m_minTrackedTwoFrames;
    const unsigned int m_minTrackedThreeFrames;

    enum TrackingStatus
    {
        Active = 0U,
        Lost = 1U
    } m_trackingStatus;
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP

#endif /* VOCPP_FEATURE_TRACKER_H */
