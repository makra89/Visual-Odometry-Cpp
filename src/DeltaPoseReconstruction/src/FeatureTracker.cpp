/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_DeltaPoseReconstruction/FeatureTracker.h>

namespace
{

void RemoveUntrackedFeatures(VOCPP::DeltaPoseReconstruction::TrackedFeatureSet& featSet, const unsigned int& in_currentFrameId)
{
    auto newEnd = std::remove_if(featSet.features.begin(), featSet.features.end(),
        [&in_currentFrameId](const VOCPP::DeltaPoseReconstruction::TrackedFeature& i) {return i.m_lastSeenFrameId < in_currentFrameId; });
    featSet.features.erase(newEnd, featSet.features.end());
};

}

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

TrackedFeatureSet FeatureTracker::ProvideDescriptionSet(unsigned int in_currentFrameId, std::vector<VOCPP::FeatureHandling::BinaryFeatureDescription>& in_currentDesc)
{
    TrackedFeatureSet featureSet{ std::vector<TrackedFeature>(), false };

    if (m_trackingStatus == TrackingStatus::Lost)
    {
        for (auto desc : in_currentDesc)
        {
            featureSet.features.push_back(TrackedFeature(desc.GetFeature().frameId, desc.GetFeature().id, desc));
        }
        
        // No valid last key set
        featureSet.isKeySet = true;
        m_currentKeyFeatureSet = featureSet;
        m_lastKeyFrameId = in_currentDesc.size() > 0U ? in_currentDesc[0].GetFeature().frameId : s_invalidFrameId;
        m_lastKeyFrameDescriptions = in_currentDesc;
        m_trackingStatus = in_currentDesc.size() > 0U ? TrackingStatus::Active : TrackingStatus::Lost;
        std::cout << "Lost" << std::endl;
    }
    else
    {
        std::cout << "Active" << std::endl;
        // Here we should have at least descriptions for the last keyFrame
        std::vector<FeatureHandling::BinaryDescriptionMatch> matches;
        bool ret = m_matcher.MatchDesriptions(in_currentDesc, m_lastKeyFrameDescriptions, matches);

        // First copy the key set history to the new feature set
        featureSet = m_currentKeyFeatureSet;

        TrackedFeatureSet missingFeatureSet{ std::vector<TrackedFeature>(), false };

        for (unsigned int it = 0U; it < matches.size(); it++)
        {
            // Check whether this feature has been observed before
            bool found = false;
            for (auto& feat : featureSet.features)
            {
                // We assume here that first frame == current frame and second frame == last frame
                if (feat.IsPresentInFrameWithFeatureId(matches[it].GetSecondFeature().frameId, matches[it].GetSecondFeature().id))
                {
                    feat.Update(matches[it].GetFirstFeature().frameId, matches[it].GetFirstFeature().id, matches[it].GetFirstDescription());
                    found = true;
                }
            }

            // If feature has not been found --> store them for later when we decide whether this will be a key set
            if (!found)
            {
                missingFeatureSet.features.push_back(TrackedFeature(matches[it].GetFirstFeature().frameId, matches[it].GetFirstFeature().id, matches[it].GetFirstDescription()));
            }
        }
        RemoveUntrackedFeatures(featureSet, in_currentFrameId);

        const unsigned int numMatchesTrackedTwoKeyFrames = matches.size();
        unsigned int numMatchesTrackedThreeKeyFrames = 0U;
        for (auto feat : m_currentKeyFeatureSet.features)
        {
            if (feat.IsPresentInFrame(m_KeyFrameIdBeforeLast))
            {
                numMatchesTrackedThreeKeyFrames++;
            }
        }
        std::cout << numMatchesTrackedTwoKeyFrames << " " << numMatchesTrackedThreeKeyFrames << " "<<in_currentFrameId<<" "<< m_KeyFrameIdBeforeLast<< std::endl;
        if (numMatchesTrackedTwoKeyFrames < m_minTrackedTwoFrames || numMatchesTrackedThreeKeyFrames < m_minTrackedThreeFrames)
        {
            featureSet.isKeySet = true;
            for (auto feat : missingFeatureSet.features)
            {
                featureSet.features.push_back(feat);
            }

            m_currentKeyFeatureSet = featureSet;
            m_lastKeyFrameDescriptions = in_currentDesc;
            m_KeyFrameIdBeforeLast = m_lastKeyFrameId;
            m_lastKeyFrameId = in_currentFrameId;
            std::cout << "Keyframe " << std::endl;
        }
        else
        {
            featureSet.isKeySet = false;
        }
    }

    return featureSet;
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP
