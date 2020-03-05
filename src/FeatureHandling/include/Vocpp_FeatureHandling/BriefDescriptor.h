/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_BRIEF_DESCRIPTOR_H
#define VOCPP_BRIEF_DESCRIPTOR_H

#include <Vocpp_Interface/Frame.h>
#include<Vocpp_FeatureHandling/Common.h>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief BRIEF descriptor, computes binary feature description
  */
class BriefDescriptor
{
public:

    /**
      * /brief Number of pairs drawn, also specifies number of bits in the returned binary description
      */
    static const uint s_numRandomPairs = 256U;

    /**
      * /brief Constructor
      */
    BriefDescriptor(const int in_randomPairDrawRadius = 50);

    /**
      * /brief Compute binary feature descriptions for provided frame. The returned description IDs will
      * be based on the provided feature IDs. Note that it might be the case that not for all features a valid
      * description can be computed
      *
      * \param[in] in_frame frame out of which the features have been extracted
      * \param[in] in_features features extracted from frame
      * \param[out] out_descriptions binary descriptions computed for provided features
      *
      * \return True if feature description for at least one keypoint successful, false otherwise
      */
    bool ComputeDescriptions(const Frame& in_frame, const std::vector<Feature>& in_features,
        std::vector<BinaryFeatureDescription>& out_descriptions);

private:
    // It is not allowed to copy the descriptor directly
    BriefDescriptor& operator=(const BriefDescriptor&);
    BriefDescriptor(const BriefDescriptor&);

    /**
      * /brief Draw random pairs among which intensities will be compared
      */
    void DrawPairs();

    /**
      * /brief Struct for storing point pairs drawn by the BRIEF descriptor
      */
    struct PointPair
    {
        float x1;
        float y1;
        float x2;
        float y2;
    };

    int m_randomPairDrawRadius; ///< radius around a feature that is used for drawing the random pairs
    std::vector<PointPair> m_pairs; ///< Point pairs used for intensity comparison
};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_BRIEF_DESCRIPTOR_H */
