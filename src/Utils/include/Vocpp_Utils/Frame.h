/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{
namespace Utils
{

/**
  * /brief Invalid frame Id marker
  */
static int s_invalidFrameId = INT_MIN;

/**
  * /brief Checks whether a frame has a valid Id
  */
static bool IsValidFrameId(const int in_frameId)
{
    return in_frameId > s_invalidFrameId ? true : false;
}

/**
  * /brief Camera Pose parametrized using a rotation matrix plus a translation
  * The rotation and translation have to satisfy: X_c = R * (X_w - T)
  * X_c are coordinates in the camera system, X_w in the world system
  */
class CameraPose
{
public:


    /**
      * /brief Constructor with provided camera orientation and camera center position
      */
    CameraPose(const cv::Mat& in_orientation, const cv::Mat& in_centerPosition, const int in_frameId) :
        m_orientation(in_orientation),
        m_camCenterPosition(in_centerPosition),
        m_validPose(true),
        m_frameId(in_frameId)
    {
    }

    /**
      * /brief Default constructor, will create invalid camera pose
      */
    CameraPose() : m_validPose(false), m_frameId(s_invalidFrameId)
    {
    }

    const cv::Mat& GetOrientation() const
    {
        return m_orientation;
    }

    const cv::Mat& GetCamCenter() const
    {
        return m_camCenterPosition;
    }

    bool IsValid() const
    {
        return m_validPose;
    }

private:

    cv::Mat m_orientation; ///< orientation of camera with respect to world coordinates
    cv::Mat m_camCenterPosition; ///< position of camera center in world coordinates
    bool m_validPose; ///< Specifies whether this pose is valid (has been constructed with a rotation + translation)
    int m_frameId; ///< frame Id this pose belongs to, necessary since CameraPose may be used without a frame object
};

/**
  * /brief Image wrapper class
  */
class Frame
{
public:

    /**
      * /brief Default constructor, will create invalid frame
      */
    Frame();
    
    /**
      * /brief Constructor with grayscale(!) image data of type CV_32F.
      * No other type will currently be accepted
      *
      * \param[in] in_grayImage image will be moved to frame object !!!
      * \param[in] in_imgId ID of frame to be created
      */
    Frame(cv::Mat&& in_grayImage, const int in_imgId);

    /**
      * /brief Get image data (const reference), either raw or grayscale
      */
    const cv::Mat& GetImage() const
    {
        return m_grayImage;
    }

    /**
      * /brief Get copy of image data, either raw or grayscale
      */
    cv::Mat GetImageCopy() const
    {
        return m_grayImage;
    }

    /**
      * /brief Returns frame ID
      */
    const int GetId() const
    {
        return m_Id;
    }

    /**
      * /brief Returns whether image frame data is valid
      */
    const bool isValid() const
    {
        return m_validFrame;
    }

private:

    cv::Mat m_grayImage; ///< grayscale image data

    int m_Id; ///< Id of the frame, must be a unique one!
    bool m_validFrame; ///< indicated whether this frame is a valid one

};

} //namespace Utils
} //namespace VOCPP

