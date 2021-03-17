/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_DELTA_CAMERA_POSE_H
#define VOCPP_DELTA_CAMERA_POSE_H

#include <Vocpp_Interface/Types.h>

namespace VOCPP
{

/**
  * /brief Translation between camera centers
  */
struct DeltaTranslation
{
public:
    double x;  ///< [arbitrary]
    double y;  ///< [arbitrary]
    double z;  ///< [arbitrary]
};

/**
  * /brief Orientation difference between camera centers (in roll, pitch, yaw angle)
  */
struct DeltaOrientation
{
public:
    double roll; ///< [rad]
    double pitch; ///< [rad]
    double yaw; ///< [rad]
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
        m_translation.x = 0.0; m_translation.y = 0.0; m_translation.z = 0.0;
        m_orientation.roll = 0.0; m_orientation.pitch = 0.0; m_orientation.yaw = 0.0;
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
