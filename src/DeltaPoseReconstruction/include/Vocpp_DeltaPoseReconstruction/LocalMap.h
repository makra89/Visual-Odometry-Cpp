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
    bool IsPresentInFrame(const unsigned int& in_frameId, const unsigned int& in_featureId) const;

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
        bool operator() (const FramePairKey& lhs, const FramePairKey& rhs)
        {
            return (lhs.currentFrameId < rhs.currentFrameId &&
                lhs.lastFrameId < rhs.lastFrameId);
        }
    };

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
    
    enum class Status {
        TrackingLost,
        TrackingActive
    };
    
    /**
      * \brief Constructor of an empty local map
      */
    LocalMap(const unsigned int in_minNumberOfTrackedLandmarks) : 
        m_landmarks(), 
        m_status(Status::TrackingLost),
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
        m_status = Status::TrackingLost;
    }
    
    /**
      * \brief Insert landmark given a feature match and a frame Id. It will be checked whether this landmark has been seen before.
      * If it has been seen before, the existing landmark will be updated.
      *
      * \param[in] in_position landmark position in WCS(!)
      * \param[in] in_match matching object, it is assumed that the frame ID of the "first" description in this match equals in_currentFrameId
      * \param[in] in_currentFrameId current frame Id
      */
    void LocalMap::InsertLandmark(const LandmarkPosition& in_position, const FeatureHandling::BinaryDescriptionMatch& in_match, const unsigned int& in_currentFrameId);

    /**
      * /brief Return tracking status of the local map
      */
    Status GetTrackingStatus()
    {
        return m_status;
    }
    
    /**
      * \brief Get all stored landmarks
      */
    const std::vector<Landmark>& GetLandmarks() const
    {
        return m_landmarks;
    }

private:

    std::vector<Landmark> m_landmarks; ///< stored landmarks
    Status m_status; ///< current landmark tracking status
    const unsigned int m_minNumberOfTrackedLandmarks; ///< Minimum number of tracked landmarks needed for setting the tracking status to active
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP


#endif /* VOCPP_LOCAL_MAP_H */
