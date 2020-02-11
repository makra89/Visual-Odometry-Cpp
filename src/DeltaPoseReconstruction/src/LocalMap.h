/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_LOCAL_MAP_H
#define VOCPP_LOCAL_MAP_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <unordered_map>
#include <iostream>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

/**
  * /brief Landmark represented by a 3D position, stores its occurence with respect to frame and keypoint IDs
  */
class Landmark
{

public:

    /**
      * /brief Constructor of a new landmark
      */
    Landmark(const cv::Vec3f& in_position, int in_currentFrameId, int in_currentKeypointId, 
        int in_lastFrameId, int in_lastKeypointId) : 
        m_lastSeenFrameId(in_currentFrameId),
        m_frameVsKeypointId(),
        m_frameIdVsPosition()
    {
        // For current frame
        m_frameVsKeypointId.insert(std::pair<int, int>(in_currentFrameId, in_currentKeypointId));
        m_frameIdVsPosition.insert(std::pair<int, cv::Vec3f>(in_currentFrameId, in_position));

        // For last frame only add that it was there
        m_frameVsKeypointId.insert(std::pair<int, int>(in_lastFrameId, in_lastKeypointId));
    }

    /**
      * /brief Update the landmark with a new occurence in a frame
      */
    void UpdateLandmark(const cv::Vec3f& in_position, int in_currentFrameId, int in_currentKeypointId)
    {
        m_frameVsKeypointId.insert(std::pair<int, int>(in_currentFrameId, in_currentKeypointId));
        m_frameIdVsPosition.insert(std::pair<int, cv::Vec3f>(in_currentFrameId, in_position));
        m_lastSeenFrameId = in_currentFrameId;
    }

    /**
      * /brief Queries whether a landmark is visible in a certain frame as a keypoint with a specific Id
      */
    bool IsPresentInFrame(int in_frameId, int in_keypointId) const
    {
        bool ret = m_frameVsKeypointId.count(in_frameId) > 0 ? true : false;

        if (ret)
        {
            ret = m_frameVsKeypointId.at(in_frameId) == in_keypointId ? true : false;
        }

        return ret;
    }

private:

    int m_lastSeenFrameId; ///< frame Id for which this landmark has been seen last 
    std::unordered_map<int, int> m_frameVsKeypointId; ///< keypoint Id of this landmark for all frames
    std::unordered_map<int, cv::Vec3f>  m_frameIdVsPosition; ///< 3D (triangulated) position of this landmark for all frames

};

/**
  * /brief Collection of Landmarks
  */
class LocalMap
{

public:
    
    /**
      * /brief Insert landmark given a cv::DMatch and a frame Id. It will be checked whether this landmark has been seen before.
      * If it has been seen before, the existing landmark will be updated
      */
    void InsertLandmark(const cv::Vec3f& in_position, const cv::DMatch& in_match, int in_currentFrameId);
    
    /**
      * /brief Get all stored landmarks
      */
    const std::vector<Landmark>& GetLandmarks() const
    {
        return m_landmarks;
    }

private:

    std::vector<Landmark> m_landmarks; ///< stored landmarks

};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP


#endif /* VOCPP_LOCAL_MAP_H */
