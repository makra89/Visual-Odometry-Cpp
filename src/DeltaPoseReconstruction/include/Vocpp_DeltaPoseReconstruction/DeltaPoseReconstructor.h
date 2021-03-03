/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_DELTA_POSE_RECONSTRUCTOR_H
#define VOCPP_DELTA_POSE_RECONSTRUCTOR_H

#include <Vocpp_Interface/Frame.h>
#include <Vocpp_Interface/DeltaCameraPose.h>
#include <Vocpp_Interface/CameraPose.h>

#include<Vocpp_FeatureHandling/OrbDetectorDescriptor.h>
#include<Vocpp_FeatureHandling/LshMatcher.h>

#include<Vocpp_DeltaPoseReconstruction/LocalMap.h>

#include <Vocpp_Utils/ImageProcessingUtils.h>

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{

namespace DeltaPoseReconstruction
{

/**
  * /brief Reconstructs epipolar geometry out of feature matches and 
  * reconstructs the delta pose between frames
  */
class DeltaPoseReconstructor
{

public:

    /**
      * /brief Constructor
      */
    DeltaPoseReconstructor();

    /**
      * /brief Destructor
      */
    ~DeltaPoseReconstructor();

    /**
      * /brief Provide next image frame to reconstructor together with a calibration matrix
      */
    bool FeedNextFrame(const Frame& in_frame, const cv::Mat1f& in_calibMat, bool in_debugOutputFlag);

    /**
      * /brief Get computed delta pose of last frame to the frame before
      */
    DeltaCameraPose GetLastDeltaPose()
    {
        return m_lastDeltaPose;
    }

    /**
      * /brief Get computed pose for last frame
      */
    CameraPose GetLastPose()
    {
        return m_lastPose;
    }

private:

    /**
      * /brief Internally used to reset all members (e.g. used during construction or when tracking has been lost)
      */
    void Reset();

    FeatureHandling::OrbDetectorDescriptor m_detectorDescriptor; ///< feature detector and descriptor
    FeatureHandling::LshMatcher m_matcher; ///< feature matcher

    LocalMap m_localMap;

    std::vector<FeatureHandling::BinaryFeatureDescription> m_descriptionsLastFrame;  ///< descriptions computed for last frame
    unsigned int m_lastFrameId;

    DeltaCameraPose m_lastDeltaPose; ///< Delta Pose calculated for last frame to the frame before
    CameraPose m_lastPose; ///< Current Pose in world coordinate system calculated for last frame
    cv::Mat1f m_lastOrientationWcs; ///< Current orientation in wcs (just stored for convenience)
    cv::Mat1f m_lastPosWcs; ///< Current position in wcs (just stored for convenience)
    Utils::ImageProjectionMatrix m_lastProjectionMat; ///< projection matrix of last frame defined as x_image = K * (Rx_wcs - T)
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP

#endif /* VOCPP_DELTA_POSE_RECONSTRUCTOR_H */