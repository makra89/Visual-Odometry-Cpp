/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_BRIEF_DESCRIPTOR_H
#define VOCPP_BRIEF_DESCRIPTOR_H

#include<Vocpp_FeatureHandling/FeatureDescriptor.h>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief BRIEF descriptor
  */
class BriefDescriptor : public FeatureDescriptor
{
public:
    /**
      * /brief Instantiate and configure a Brief descriptor
      *
      * /returns Pointer to object if successful, NULL otherwise. Caller takes ownership of object.
      */
    static BriefDescriptor* CreateInstance(const uint32_t in_randomPairDrawRadius = 50, const uint32_t in_numRandomPairs = 256);

    /**
      * /brief Compute descriptions for provided frame
      *
      * \param[in,out] inout_frame frame for which to compute the descriptions, features must be present!
      * \return True if feature description for at least one feature successfull, false otherwise
      */
    virtual bool ComputeDescriptions(Utils::Frame& inout_frame);

private:
    // It is not allowed to copy or instantiate the descriptor directly
    BriefDescriptor::BriefDescriptor();
    BriefDescriptor& operator=(const BriefDescriptor&);
    BriefDescriptor(const BriefDescriptor&);

    /**
      * /brief Draw random pairs among which intensities will be compared
      */
    void DrawPairs();

    uint32_t m_randomPairDrawRadius; ///< radius around a feature that is used for drawing the random pairs
    uint32_t m_numRandomPairs; ///< number of random pairs used
    std::vector<cv::Mat> m_pairs; ///< Point pairs used for intensity comparison
};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_BRIEF_DESCRIPTOR_H */