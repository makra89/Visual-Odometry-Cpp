/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_FEATURE_DESCRIPTOR_H
#define VOCPP_FEATURE_DESCRIPTOR_H

#include<Vocpp_Interface/Frame.h>
#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief Generic feature descriptor class. All feature descriptor implementations have to
  * derive from this class
  */
class FeatureDescriptor
{
public:
    /**
      * /brief Compute keypoints descriptions for provided frame
      *
      * \param[in] in_frame frame out of which the keypoints have been extracted
      * \param[in] in_keypoints keypoints extracted from frame
      * \param[out] out_descriptions descriptions computed for provided keypoints
      * \param[out] out_validKeypoints keypoints for which a description could be computed

      * \return True if feature description for at least one keypoint successfull, false otherwise
      */
    virtual bool ComputeDescriptions(const Frame& in_frame, const std::vector<cv::KeyPoint>& in_keypoints,
        std::vector<cv::Mat>& out_descriptions, std::vector<cv::KeyPoint>& out_validKeypoints) = 0;
};

/**
  * /brief Instantiate a feature descriptor given a descriptor name.
  *
  * \param[in] in_descriptorName name of descriptor
  * \returns pointer to feature descriptor object if successful, NULL otherwise. Caller takes
  * ownership of object
  */
FeatureDescriptor* InstantiateFeatureDescriptor(std::string in_descriptorName);


} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_FEATURE_DESCRIPTOR_H */