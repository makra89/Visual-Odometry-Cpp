/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_CAMERA_POSE_H
#define VOCPP_CAMERA_POSE_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

namespace VOCPP
{

/**
  * /brief Translation to world coordinate center
  */
struct Translation
{
public:
    float x;  ///< [arbitrary]
    float y;  ///< [arbitrary]
    float z;  ///< [arbitrary]
};

/**
  * /brief Orientation difference between camera and world coordinate system (in roll, pitch, yaw angle)
  */
struct Orientation
{
public:
    float roll; ///< [rad]
    float pitch; ///< [rad]
    float yaw; ///< [rad]
};

/**
  * /brief Absolute camera pose in the world coordinate system
  */
class CameraPose
{
public:


    /**
      * /brief Constructor with provided camera orientation and camera center position
      */
    CameraPose(const Translation& in_translation, const Orientation& in_orientation) :
        m_translation(in_translation),
        m_orientation(in_orientation)
    {
    }

    /**
      * /brief Default constructor, longitudinal axis of body system aligned with WCS X axis
      */
    CameraPose()
    {
        m_translation = { 0.0F, 0.0F, 0.0F };
        m_orientation = { 0.0F, 0.0F, 0.0F };
    }

    const Orientation& GetOrientation() const
    {
        return m_orientation;
    }

    const Translation& GetCamCenterTranslation() const
    {
        return m_translation;
    }

private:

    Translation m_translation; ///< translation of camera center to world coordinate center in world coordinate system
    Orientation m_orientation; ///< orientation of the camera with respect to the world coordinate system
};

} //namespace VOCPP

#endif /* VOCPP_CAMERA_POSE_H */
