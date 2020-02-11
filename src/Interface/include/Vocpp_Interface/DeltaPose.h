/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_DELTA_POSE_H
#define VOCPP_DELTA_POSE_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

namespace VOCPP
{

/**
  * /brief Delta camera pose parametrized using a rotation matrix plus a translation
  * The rotation and translation have to satisfy: X_2 = R * (X_1 - T)
  * X_1 are coordinates in the starting camera system, X_2 in end camera system
  */
class DeltaPose
{
public:


    /**
      * /brief Constructor with provided camera orientation and camera center position
      */
    DeltaPose(const cv::Mat& in_deltaOrientation, const cv::Mat& in_translation) :
        m_deltaOrientation(in_deltaOrientation),
        m_translation(in_translation),
        m_validPose(true)
    {
    }

    /**
      * /brief Default constructor, will create invalid camera pose
      */
    DeltaPose() : m_validPose(false)
    {
    }

    const cv::Mat& GetDeltaOrientation() const
    {
        return m_deltaOrientation;
    }

    const cv::Mat& GetCamCenterTranslation() const
    {
        return m_translation;
    }

    bool IsValid() const
    {
        return m_validPose;
    }

private:

    cv::Mat m_deltaOrientation; ///< delta orientation of the camera with respect to the starting camera system
    cv::Mat m_translation; ///< translation of first to second camera center
    bool m_validPose; ///< Specifies whether this delta pose is valid (has been constructed with a rotation + translation)
};

} //namespace VOCPP

#endif /* VOCPP_DELTA_POSE_H */
