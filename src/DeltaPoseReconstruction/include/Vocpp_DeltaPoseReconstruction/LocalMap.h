/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_LOCAL_MAP_H
#define VOCPP_LOCAL_MAP_H

#include <Vocpp_FeatureHandling/Common.h>
#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <unordered_map>
#include <iostream>
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
    float x;
    float y;
    float z;
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
    Landmark(const unsigned int& in_currentFrameId, const unsigned int& in_currentFeatureId, 
        const unsigned int& in_lastFrameId, const unsigned int& in_lastFeatureId, const LandmarkPosition& in_position) :
        m_lastSeenFrameId(in_currentFrameId),
        m_frameVsFeatureId(),
        m_framePairVsPosition()
    {
        m_frameVsFeatureId.insert(std::pair<int, int>(in_lastFrameId, in_lastFeatureId));
        m_frameVsFeatureId.insert(std::pair<int, int>(in_currentFrameId, in_currentFeatureId));

        // Add triangulated position
        FramePairKey keyPair{ in_currentFrameId, in_lastFrameId};
        m_framePairVsPosition.insert(std::pair<FramePairKey, LandmarkPosition>(keyPair, in_position));
    }

    /**
      * \brief Update the landmark with a new occurence in a frame
      */
    void UpdateLandmark(const unsigned int& in_currentFrameId, const unsigned int& in_currentFeatureId,
        const unsigned int& in_lastFrameId, const unsigned int& in_lastFeatureId, const LandmarkPosition& in_position);

    /**
      * \brief Queries whether a landmark is visible in a certain frame as a feature with a specific Id
      */
    bool IsPresentInFrameWithFeatureId(const unsigned int& in_frameId, const unsigned int& in_featureId) const;

    /**
      * \brief Queries whether a landmark is visible in a certain frame
      */
    bool IsPresentInFrame(const unsigned int& in_frameId) const;

    const unsigned int GetLastSeenFrameId() const
    {
        return m_lastSeenFrameId;
    }

    /**
      * \brief Helper struct for holding two keys for a map
      */
    struct FramePairKey
    {
        unsigned int currentFrameId;
        unsigned int lastFrameId;
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

    bool GetFramePairPosition(const FramePairKey& in_pairKey, LandmarkPosition& out_position);

private:
    int m_lastSeenFrameId; ///< frame Id for which this landmark has been seen last 
    std::map<int, int> m_frameVsFeatureId; ///< feature Ids of this landmark for all frames
    std::map<FramePairKey, LandmarkPosition, CompFramePairKey> m_framePairVsPosition; ///< frame Id pair vs triangulated position
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
    LocalMap(const unsigned int in_minNumberOfTrackedLandmarks) : 
        m_landmarks(), 
        m_minNumberOfTrackedLandmarks(in_minNumberOfTrackedLandmarks)
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
    void InsertLandmarks(const std::vector<LandmarkPosition>& in_positions, const std::vector<FeatureHandling::BinaryDescriptionMatch>& in_matches, const unsigned int& in_currentFrameId);

    void RemoveUntrackedLandmarks(const unsigned int& in_currentFrameId);
    
    /**
      * \brief Get all stored landmarks
      */
    const std::vector<Landmark>& GetLandmarks() const
    {
        return m_landmarks;
    }

private:

    void ComputeRelativeScale(const unsigned int& in_currentFrameId);

    std::vector<Landmark> m_landmarks; ///< stored landmarks
    const unsigned int m_minNumberOfTrackedLandmarks; ///< Minimum number of tracked landmarks needed for setting the tracking status to active
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP


#endif /* VOCPP_LOCAL_MAP_H */
