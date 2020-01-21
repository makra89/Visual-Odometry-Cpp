/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

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
        const uint32_t in_localMaxDistance = 10U, const uint32_t in_subPixelCalculationDistance = 5U);

    /**
    * /brief Extract features from a provided grayscale(!!) image frame. Keypoints will be added to the provided image frame
    *
    * \param[in, out] inout_frame grayscale image frame from which features shall be extracted
    * \return True if feature detection successfull, false otherwise
    */
    virtual bool ExtractKeypoints(Utils::Frame& inout_frame) override;

private:
    // It is not allowed to copy or instantiate the detector directly
    HarrisEdgeDetector::HarrisEdgeDetector();
    HarrisEdgeDetector& operator=(const HarrisEdgeDetector&);
    HarrisEdgeDetector(const HarrisEdgeDetector&);

    double m_relResponseThresh; ///< relative response threshold used to filter out insignificant features [with respect to maximum response]
    double m_k; ///< k factor used for Harris response calculation, see literature
    uint32_t m_localMaxDistance; ///< minimum distance of reported features [pixel] 
    uint32_t m_subPixelCalculationDistance; ///< radius that is taken into account for subPixel feature position calculation [pixels]
    
    cv::Mat m_smoothingKernel; ///< kernel used for smoothing the gradients

};

} //namespace FeatureHandling
} //namespace VOCPP

