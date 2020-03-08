/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_HARRIS_EDGE_DETECTOR_H
#define VOCPP_HARRIS_EDGE_DETECTOR_H

#include <Vocpp_Interface/Frame.h>
#include <Vocpp_FeatureHandling/Common.h>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * \brief Feature detector class that uses the Harris Edge detection algorithm.
  * The detector will return the features sorted according to their response. 
  * A maximum number of returned features can be specified.
  */
class HarrisEdgeDetector
{
public:
    /**
      * \brief Constructor
      */
    HarrisEdgeDetector(const int in_maxNumFeatures = 150, const float in_k = 0.06, const std::string& in_kernelName = "window",
        const int in_localMaxDistance = 10U, const int in_subPixelCalculationDistance = 5U);

    /**
      * \brief Extract features from a provided grayscale image frame.
      *
      * \param[in] in_frame image frame from which features shall be extracted
      * \param[out] out_features features extracted from the frame
      * \return True if at least one feature has been detected, false otherwise
      */
    bool ExtractFeatures(const Frame& in_frame, std::vector<Feature>& out_features);

    /**
      * \brief Compute Harris score for a single pixel, the Harris response is averaged over a patch with size in_blockSize
      *
      * \return Harris score if successful, -1.0F if not (edge would have a score > 0)
      */
    float ComputeScore(const cv::Mat1f& in_img, const int& in_centerX, const int& in_centerY, const int& in_blockSize);

private:
    // It is not allowed to copy the detector directly
    HarrisEdgeDetector& operator=(const HarrisEdgeDetector&);
    HarrisEdgeDetector(const HarrisEdgeDetector&);

    int m_maxNumFeatures; ///< maximum number of returned features
    float m_k; ///< k factor used for Harris response calculation, see literature
    int m_localMaxDistance; ///< minimum distance of reported features [pixel] 
    int m_subPixelCalculationDistance; ///< radius that is taken into account for subPixel feature position calculation [pixels]
    
    cv::Mat1f m_smoothingKernel; ///< kernel used for smoothing the gradients

};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_HARRIS_EDGE_DETECTOR_H */
