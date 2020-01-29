/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_BRUTE_FORCE_MATCHER_H
#define VOCPP_BRUTE_FORCE_MATCHER_H

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>
#include<Vocpp_FeatureHandling/FeatureMatcher.h>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief Brute force binary feature matcher class
  */
class BruteForceMatcher : public FeatureMatcher
{
public:
    
    /**
      * /brief Instantiate and configure a Brute Force matcher
      *
      * /returns Pointer to object if successful, NULL otherwise. Caller takes ownership of object.
      */
    static BruteForceMatcher* CreateInstance(const uint32_t in_maxDistance=25U);

    /**
      * /brief Match feature descriptions in provided frame and return matches. Features and descriptions have to be present!
      * Features in first frame will be searched in second frame.
      *
      * \param[in, out] inout_frame1 first frame to match
      * \param[in] inout_frame2 second frame to match
      * \param[in] in_appendMatches specifies whether matches shall be appended to match vectors
      * \return True if feature matching successfull, false otherwise
      */
    virtual bool matchDesriptions(Utils::Frame& inout_frame1, const Utils::Frame& in_frame2, const bool in_appendMatches = false) override;

private:
    // It is not allowed to copy or instantiate the detector directly
    BruteForceMatcher::BruteForceMatcher();
    BruteForceMatcher& operator=(const BruteForceMatcher&);
    BruteForceMatcher(const BruteForceMatcher&);

    /**
      * /brief Compute Hamming Distance for two descriptors
      *
      * \param[ín] in_left first (binary) description
      * \param[ín] in_right second (binary) description
      * \return True distance of descriptions
      */
    uint32_t ComputeHammingDistance(const cv::Mat& in_left, const cv::Mat& in_right);

    uint32_t m_maxDistance; ///< maximum Hamming distance between binary descriptions for reporting a match


};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_BRUTE_FORCE_MATCHER_H */