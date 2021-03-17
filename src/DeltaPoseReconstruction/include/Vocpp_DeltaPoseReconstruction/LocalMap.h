/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_LOCAL_MAP_H
#define VOCPP_LOCAL_MAP_H

#include <Vocpp_Interface/Frame.h>
#include <Vocpp_FeatureHandling/Common.h>
#include <opencv2/core/core.hpp>
#include <unordered_map>

#include <map>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

/**
* /brief 3D Landmark position in world coordinates
*/
struct LandmarkPosition
{
    double x;
    double y;
    double z;
};

/**
  * \brief Landmark represented by a 3D position, stores its occurence with respect to frame and keypoint IDs
  * Note: A landmark is "visible" in a frame when it can be triangulated in this frame with respect to another frame.
  *       Therefore, when the landmark is beeing constructed it has been "visible" in at least to frames!
  */
class Landmark
{

public:
    /**
      * \brief Constructor of a new landmark
      */
    Landmark(const uint32_t& in_currentFrameId, const uint32_t& in_currentFeatureId, 
        const uint32_t& in_lastFrameId, const uint32_t& in_lastFeatureId, const LandmarkPosition& in_position) :
        m_lastSeenFrameId(in_currentFrameId),
        m_frameVsFeatureId(),
        m_framePairVsScaled(),
        m_framePairVsPosition()
    {
        m_frameVsFeatureId.insert(std::pair<uint32_t, uint32_t>(in_lastFrameId, in_lastFeatureId));
        m_frameVsFeatureId.insert(std::pair<uint32_t, uint32_t>(in_currentFrameId, in_currentFeatureId));

        // Add triangulated position
        FramePairKey keyPair = { in_currentFrameId, in_lastFrameId};
        m_framePairVsPosition.insert(std::pair<FramePairKey, LandmarkPosition>(keyPair, in_position));

        // For the first frame pair we cannot apply any relative scale --> always indicate that it has been scaled
        m_framePairVsScaled.insert(std::pair<FramePairKey, bool>(keyPair, true));
    }

    /**
      * \brief Update the landmark with a new occurence in a frame
      */
    void UpdateLandmark(const uint32_t& in_currentFrameId, const uint32_t& in_currentFeatureId,
        const uint32_t& in_lastFrameId, const uint32_t& in_lastFeatureId, const LandmarkPosition& in_position);

    /**
      * \brief Queries whether a landmark is visible in a certain frame as a feature with a specific Id
      */
    bool IsPresentInFrameWithFeatureId(const uint32_t& in_frameId, const uint32_t& in_featureId) const;

    /**
      * \brief Queries whether a landmark is visible in a certain frame
      */
    bool IsPresentInFrame(const uint32_t& in_frameId) const;

    const uint32_t GetLastSeenFrameId() const
    {
        return m_lastSeenFrameId;
    }

    /**
      * \brief Helper struct for holding two keys for a map
      */
    struct FramePairKey
    {
        uint32_t currentFrameId;
        uint32_t lastFrameId;
    };

    /**
      * \brief Comparison functor for map using FramePairKey as key
      */
    struct CompFramePairKey
    {
        bool operator() (const FramePairKey& lhs, const FramePairKey& rhs) const
        {
            return (lhs.currentFrameId < rhs.currentFrameId);
        }
    };

    bool GetFramePairPosition(const FramePairKey& in_pairKey, LandmarkPosition& out_position) const;

    bool HasBeenScaledForFramePair(const FramePairKey& in_key) const;

    bool RescalePosition(const FramePairKey& in_pairKey, const double& in_scale);

private:
    uint32_t m_lastSeenFrameId; ///< frame Id for which this landmark has been seen last 
    std::map<uint32_t, uint32_t> m_frameVsFeatureId; ///< feature Ids of this landmark for all frames
    std::map<FramePairKey,bool,CompFramePairKey> m_framePairVsScaled; ///< indicates whether a landmark has already been scaled for certain frame pair
    std::map<FramePairKey,LandmarkPosition,CompFramePairKey> m_framePairVsPosition; ///< frame Id pair vs triangulated position
};

/**
  * \brief Collection of Landmarks
  */
class LocalMap
{

public:
    /**
      * \brief Constructor of an empty local map
      */
    LocalMap(const uint32_t in_minNumberOfTrackedLandmarks) : 
        m_landmarks(), 
        m_minNumberOfTrackedLandmarks(in_minNumberOfTrackedLandmarks),
        m_lastFrameId(s_invalidFrameId),
        m_validTrackedLandmarks(0U),
        m_lastRelativeScale(0.0)
    {
        Reset();
    }

    /**
      * /brief Destructor
      */
    ~LocalMap();

    /**
      * /brief Reset local map
      */
    void Reset()
    {
        m_landmarks.clear();
    }
    
    /**
      * \brief Insert landmarks given feature matches and a frame Id. It will be checked whether these landmarks have been seen before.
      * If they have been seen before, the existing landmarks will be updated. Elements with same index in position and match vector must
      * correspond to each other. Furthermore, it is expected, that no more landmarks will be added for this frame!
      *
      * \param[in] in_positions landmark positions in WCS(!)
      * \param[in] in_matches matching object, it is assumed that the frame ID of the "first" description in this match equals in_currentFrameId
      * \param[in] in_currentFrameId current frame Id
      */
    void InsertLandmarks(const std::vector<LandmarkPosition>& in_positions, const std::vector<FeatureHandling::BinaryDescriptionMatch>& in_matches, const uint32_t& in_currentFrameId);

    bool GetLastRelativeScale(const uint32_t& in_lastFrameId, const uint32_t& in_currentFrameId, double& out_scale) const;
    
    /**
      * \brief Get all stored landmarks
      */
    const std::vector<Landmark>& GetLandmarks() const
    {
        return m_landmarks;
    }

private:

    void RemoveUntrackedLandmarks(const uint32_t& in_currentFrameId);

    void ComputeRelativeScale(const uint32_t& in_currentFrameId);

    std::vector<Landmark> m_landmarks; ///< stored landmarks
    const uint32_t m_minNumberOfTrackedLandmarks; ///< Minimum number of tracked landmarks needed for setting the tracking status to active
    uint32_t m_lastFrameId;
    uint32_t m_validTrackedLandmarks;
    double m_lastRelativeScale;
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP


#endif /* VOCPP_LOCAL_MAP_H */
