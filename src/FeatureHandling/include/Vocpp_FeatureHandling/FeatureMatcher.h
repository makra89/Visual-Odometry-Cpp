/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_FEATURE_MATCHER_H
#define VOCPP_FEATURE_MATCHER_H

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief Generic feature matcher class. All feature matcher implementations have to
  * derive from this class
  */
class FeatureMatcher
{
public:
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
        const int& in_secondFrameId, std::vector<cv::DMatch>& out_matches) = 0;
};

/**
  * /brief Instantiate a feature matcher given a matcher name.
  *
  * \param[in] in_matcherName name of feature matcher
  * \returns pointer to feature matcher object if successful, NULL otherwise. Caller takes
  * ownership of object
  */
FeatureMatcher* InstantiateFeatureMatcher(std::string in_matcherName);


} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_FEATURE_MATCHER_H */
