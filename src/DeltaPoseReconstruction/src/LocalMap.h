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
* /brief 3D Landmark position in world coordinates
*/
struct LandmarkPosition
{
    float x;
    float y;
    float z;
};

/**
  * /brief Landmark represented by a 3D position, stores its occurence with respect to frame and keypoint IDs
  */
class Landmark
{

public:

    /**
      * /brief Constructor of a new landmark
      */
    Landmark(int in_currentFrameId, int in_currentKeypointId, const cv::Mat1f& in_currentProjectionMat)// : 
        //m_lastSeenFrameId(in_currentFrameId),
        //m_frameVsKeypointId(),
        //m_framePairVsPosition()
    {
       // m_frameVsKeypointId.insert(std::pair<int, int>(in_currentFrameId, in_currentKeypointId));
       // m_frameVsProjectionMat.insert(std::pair<int, cv::Mat1f>(in_currentFrameId, in_currentProjectionMat));
    }

    /**
      * /brief Update the landmark with a new occurence in a frame
      */
    void UpdateLandmark(const LandmarkPosition& in_position, int in_currentFrameId, int in_currentKeypointId)
    {
        /*m_frameVsKeypointId.insert(std::pair<int, int>(in_currentFrameId, in_currentKeypointId));
        m_frameIdVsPosition.insert(std::pair<int, LandmarkPosition>(in_currentFrameId, in_position));
        m_lastSeenFrameId = in_currentFrameId;
        std::cout << "Found keypoint" << std::endl;
        for (auto bla : m_frameVsKeypointId)
        {
            std::cout << bla.first << " " << bla.second << std::endl;
        }
        for (auto bla : m_frameIdVsPosition)
        {
            std::cout << bla.first << std::endl;
        }*/
    }

    /**
      * /brief Queries whether a landmark is visible in a certain frame as a keypoint with a specific Id
      */
    bool IsPresentInFrame(int in_frameId, int in_keypointId) const
    {
       // bool ret = m_frameVsKeypointId.count(in_frameId) > 0 ? true : false;

       // if (ret)
       // {
        //    ret = m_frameVsKeypointId.at(in_frameId) == in_keypointId ? true : false;
      //  }

        return true;
    }

private:

    int m_lastSeenFrameId; ///< frame Id for which this landmark has been seen last 
    //std::unordered_map<int, int> m_frameVsKeypointId; ///< keypoint Id of this landmark for all frames
    //std::unordered_map<int, cv::Mat1f> m_frameVsProjectionMat;
    //std::unordered_map<std::pair<int, int>, LandmarkPosition> m_framePairVsPosition;
};

/**
  * /brief Collection of Landmarks
  */
class LocalMap
{

public:
    
    
    enum class Status {
        TrackingLost,
        NotTriangulated,
        TrackingActive
    };
    
    /**
      * /brief Constructor of an empty local map
      */
    LocalMap(const int in_minNumberOfTrackedLandmarks) : 
        m_landmarks(), 
        m_status(Status::TrackingLost),
        m_minNumberOfTrackedLandmarks(in_minNumberOfTrackedLandmarks)
    {
    }

    /**
      * /brief Destructor
      */
    ~LocalMap();
    
    void Initialize(const std::vector<cv::KeyPoint>& in_keypoints, const cv::Mat& in_projectionMat, const int in_currentFrameId);
    
    /**
      * /brief Insert landmark given a cv::DMatch and a frame Id. It will be checked whether this landmark has been seen before.
      * If it has been seen before, the existing landmark will be updated.
      */
    //void InsertSetOfMatches(const std::vector<cv::DMatch>& in_match, const int in_currentFrameId);

    /**
      * /brief Return tracking status of the local map
      */
    Status GetTrackingStatus()
    {
        return m_status;
    }
    
    /**
      * /brief Get all stored landmarks
      */
    const std::vector<Landmark>& GetLandmarks() const
    {
        return m_landmarks;
    }

private:

    std::vector<Landmark> m_landmarks; ///< stored landmarks
    Status m_status; ///< current landmark tracking status
    const int m_minNumberOfTrackedLandmarks; ///< Minimum number of tracked landmarks needed for setting the tracking status to active
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP


#endif /* VOCPP_LOCAL_MAP_H */
