/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_ORIENTED_FAST_DETECTOR_H
#define VOCPP_ORIENTED_FAST_DETECTOR_H

#include <Vocpp_Interface/Frame.h>
#include <Vocpp_FeatureHandling/Common.h>
#include <Vocpp_FeatureHandling/HarrisEdgeDetector.h>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief Oriented Fast Detector which is part of an ORB detector/descriptor
  * This detector searches edges in an image, computes the Harris score for each of them
  * and calculates the feature orientation using the intensity centroid in a patch around the image
  */
class OrientedFastDetector
{
public:
    /**
      * /brief Constructor
      */
    OrientedFastDetector(const float& in_relTresh=0.2, const int& in_numPixelsAboveThresh=12,
        const int& in_harrisBlockSize=3);

    /**
      * /brief Extract features from a provided grayscale image frame.
      *
      * \param[in] in_frame image frame from which features shall be extracted
      * \param[in] in_maxNumFeatures maximum number of features returned
      * \param[out] out_features features extracted from the frame
      * \return True if at least one feature has been detected, false otherwise
      */
    bool ExtractFeatures(const Frame& in_frame, const int& in_maxNumFeatures, std::vector<Feature>& out_features);

private:
    // It is not allowed to copy the detector directly
    OrientedFastDetector& operator=(const OrientedFastDetector&);
    OrientedFastDetector(const OrientedFastDetector&);

    /**
      * /brief Compute the FAST score for the current pixel (in_coordX / in_coordY), this score can be used in the following way:
      * The score of an edge should be higher than m_numPixelsAboveThresh 
      * Internally first 4 edge pixels are checked and then CheckAll() is called
      * \return score, can be compared against m_numPixelsAboveThresh
      */
    int CheckIntensities(const cv::Mat1f& in_image, const int& in_coordX, const int& in_coordY, const int& in_imgWidth, const int& in_imgHeight);
    
    /**
      * /brief Called by CheckIntensities()
      */
    int CheckAll(const cv::Mat1f& in_image, const int& in_coordX, const int& in_coordY);

    float m_relTresh; ///< relative value for pixel intensity comparison
    int m_numPixelsAboveThresh; ///< necessary number of pixels above or below threshold for a detection
    int m_harrisBlockSize; ///< size of patch over which harris detector averages gradients (has to be odd number)
    HarrisEdgeDetector m_harrisDetector; ///< Harris Edge detector, used to compute feature scores

    static const int s_featureSize; ///< path size which is taken into account during feature detection and orientation determination
};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_ORIENTED_FAST_DETECTOR_H */
