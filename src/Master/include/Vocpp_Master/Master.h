/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_MASTER_H
#define VOCPP_MASTER_H

#include<Vocpp_Interface/Frame.h>
#include<Vocpp_Interface/DeltaPose.h>

#include<Vocpp_DeltaPoseReconstruction/DeltaPoseReconstructor.h>
#include<Vocpp_Calibration/MonoCameraCalibration.h>
#include<Vocpp_Calibration/CalibrationModule.h>

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{
namespace Master
{
   
/**
  * /brief Master class for calculation of delta poses
  *
  * Until now it is only possible to match features and draw those matches
  */
class Master
{

public:
    
    /**
      * /brief Constructor
      */
    Master();

    /**
      * /brief Destructor
      */
    ~Master();
    
    /**
      * /brief Provide next image frame to master
      */
    bool FeedNextFrame(const Frame& in_frame);

    /**
      * /brief Get computed delta pose of last frame to the frame before
      * Be sure to check the delta pose for validity!
      */
    DeltaPose GetLastDeltaPose();

    /**
      * /brief Load a mono camera calibration
      */
    bool LoadCalibration(const Calibration::MonoCameraCalibration& in_monoCalibration);

private:

    DeltaPoseReconstruction::DeltaPoseReconstructor m_reconstructor; /// reconstructor which provides delta poses between consecutive frames
    Calibration::CalibrationModule m_calibModule; ///< calibration module which stores a loaded calibration (and in future computes it)

};

} //namespace Master
} //namespace VOCPP

#endif /* VOCPP_MASTER_H */
