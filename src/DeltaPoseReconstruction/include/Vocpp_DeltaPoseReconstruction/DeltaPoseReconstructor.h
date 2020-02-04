/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_DELTA_POSE_RECONSTRUCTOR_H
#define VOCPP_DELTA_POSE_RECONSTRUCTOR_H

#include <Vocpp_Utils/Frame.h>

#include<Vocpp_FeatureHandling/FeatureDescriptor.h>
#include<Vocpp_FeatureHandling/FeatureMatcher.h>
#include<Vocpp_FeatureHandling/FeatureDetector.h>

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{

namespace DeltaPoseReconstruction
{

class EpipolarModel;
class RansacOptimizer;

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
      * /brief Provide next image frame to reconstructor together with a calibration matrix
      */
    bool FeedNextFrame(Utils::Frame& in_frame, const cv::Mat& in_calibMat);

private:

    FeatureHandling::FeatureDetector* m_detector; ///< feature detector
    FeatureHandling::FeatureDescriptor* m_descriptor; ///< feature descriptor
    FeatureHandling::FeatureMatcher* m_matcher; ///< feature matcher

    RansacOptimizer* m_optimizer;
    std::vector<EpipolarModel*> m_epiPolModels;

    Utils::Frame m_lastFrame;  ///< last processed frame (fed via DeltaPoseReconstructor::FeedNextFrame(Utils::Frame& in_frame)
};

} //namespace DeltaPoseReconstruction
} //namespace VOCPP

#endif /* VOCPP_DELTA_POSE_RECONSTRUCTOR_H */