/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_DELTA_CAMERA_POSE_H
#define VOCPP_DELTA_CAMERA_POSE_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

namespace VOCPP
{

/**
  * /brief Translation between camera centers
  */
struct DeltaTranslation
{
public:
    float x;  ///< [arbitrary]
    float y;  ///< [arbitrary]
    float z;  ///< [arbitrary]
};

/**
  * /brief Orientation difference between camera centers (in roll, pitch, yaw angle)
  */
struct DeltaOrientation
{
public:
    float roll; ///< [rad]
    float pitch; ///< [rad]
    float yaw; ///< [rad]
};

/**
  * /brief Delta camera pose parametrized using a delta orientation plus a translation
  * The rotation and translation are given in the body system of the first camera center
  */
class DeltaCameraPose
{
public:


    /**
      * /brief Constructor with provided camera orientation and camera center position
      */
    DeltaCameraPose(const DeltaTranslation& in_deltaTrans, const DeltaOrientation& in_deltaOrientation) :
        m_translation(in_deltaTrans),
        m_orientation(in_deltaOrientation)
    {
    }

    /**
      * /brief Default constructor
      */
    DeltaCameraPose()
    {
        m_translation = { 0.0F, 0.0F, 0.0F };
        m_orientation = { 0.0F, 0.0F, 0.0F };
    }

    const DeltaOrientation& GetDeltaOrientation() const
    {
        return m_orientation;
    }

    const DeltaTranslation& GetCamCenterTranslation() const
    {
        return m_translation;
    }

private:

    DeltaTranslation m_translation; ///< translation of first to second camera center in body system
    DeltaOrientation m_orientation; ///< delta orientation of the camera with respect to the starting camera system 
};

} //namespace VOCPP

#endif /* VOCPP_DELTA_CAMERA_POSE_H */
