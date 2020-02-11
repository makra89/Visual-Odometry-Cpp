/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_HARRIS_EDGE_DETECTOR_H
#define VOCPP_HARRIS_EDGE_DETECTOR_H

#include<Vocpp_FeatureHandling/FeatureDetector.h>

namespace VOCPP
{
namespace FeatureHandling
{

class HarrisEdgeDetector : public FeatureDetector
{
public:
    /**
      * /brief Instantiate and configure a Harris edge detector
      *
      * /returns Pointer to object if successful, NULL otherwise. Caller takes ownership of object.
      */
    static HarrisEdgeDetector* CreateInstance(const double in_relResponseThresh = 0.01, const double in_k = 0.04, const std::string& in_kernelName = "window",
        const int in_localMaxDistance = 10U, const int in_subPixelCalculationDistance = 5U);

    /**
      * /brief Extract features from a provided image frame. Implementations might place certain
      * restrictions to the image (like being grayscale).
      *
      * \param[in] in_frame image frame from which features shall be extracted
      * \param[out] out_keypoints keypoints extracted from the frame
      * \return True if feature detection successfull, false otherwise
      */
    virtual bool ExtractKeypoints(const Frame& in_frame, std::vector<cv::KeyPoint>& out_keypoints) override;

private:
    // It is not allowed to copy or instantiate the detector directly
    HarrisEdgeDetector::HarrisEdgeDetector();
    HarrisEdgeDetector& operator=(const HarrisEdgeDetector&);
    HarrisEdgeDetector(const HarrisEdgeDetector&);

    double m_relResponseThresh; ///< relative response threshold used to filter out insignificant features [with respect to maximum response]
    double m_k; ///< k factor used for Harris response calculation, see literature
    int m_localMaxDistance; ///< minimum distance of reported features [pixel] 
    int m_subPixelCalculationDistance; ///< radius that is taken into account for subPixel feature position calculation [pixels]
    
    cv::Mat m_smoothingKernel; ///< kernel used for smoothing the gradients

};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_HARRIS_EDGE_DETECTOR_H */
