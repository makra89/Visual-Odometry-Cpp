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
    static BruteForceMatcher* CreateInstance(const int in_maxDistance=50U);

    /**
      * /brief Compares keypoint descriptions and return matches.
      * Descriptions in first set of provided descriptions will be compared to the ones in the second set
      * The fields of the output cv::DMatch structs have the following meaning:
      *     - queryIdx: Id of the first description (Id == index in vector)
      *     - trainIdx: Id of the second description (Id == index in vector)
      *     - imgIdx: Id of the frame belonging to the second frame (map description1 --> description2)
      *     - distance: distance measure of descriptions
      *
      * \param[in] in_descriptions1 first set of keypoint descriptions
      * \param[in] in_descriptions2 second set of keypoint descriptions
      * \param[in] in_secondFrameId Id of the frame belonging to the second set of descriptions
      * \param[out] out_matches matches between description sets
      * \return True if feature matching successfull, false otherwise
      */
    virtual bool MatchDesriptions(const std::vector<cv::Mat>& in_descriptions1, const std::vector<cv::Mat>& in_descriptions2,
        const int& in_secondFrameId, std::vector<cv::DMatch>& out_matches) override;

private:
    // It is not allowed to copy or instantiate the detector directly
    BruteForceMatcher();
    BruteForceMatcher& operator=(const BruteForceMatcher&);
    BruteForceMatcher(const BruteForceMatcher&);

    /**
      * /brief Compute Hamming Distance for two descriptors
      *
      * \param[ín] in_left first (binary) description
      * \param[ín] in_right second (binary) description
      * \return True distance of descriptions
      */
    int ComputeHammingDistance(const cv::Mat& in_left, const cv::Mat& in_right);

    int m_maxDistance; ///< maximum Hamming distance between binary descriptions for reporting a match


};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_BRUTE_FORCE_MATCHER_H */
