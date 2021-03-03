/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_MASTER_H
#define VOCPP_MASTER_H

#include<Vocpp_Interface/Frame.h>
#include<Vocpp_Interface/DeltaCameraPose.h>
#include<Vocpp_Interface/CameraPose.h>

#include<Vocpp_DeltaPoseReconstruction/DeltaPoseReconstructor.h>
#include<Vocpp_Calibration/MonoCameraCalibration.h>
#include<Vocpp_Calibration/CalibrationModule.h>

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
      * /brief Get computed delta pose in body frame of last frame to the frame before
      */
    DeltaCameraPose GetLastDeltaPose();

    /**
      * /brief Get current pose in world coordinate system
      */
    CameraPose GetLastPose();

    /**
      * /brief Activate debug output (like drawing current frame + matches)
      */
    void ActivateDebugOutput();

    /**
      * /brief Deactivate debug output
      */
    void DeactivateDebugOutput();

    /**
      * /brief Load a mono camera calibration
      */
    bool LoadCalibration(const Calibration::MonoCameraCalibration& in_monoCalibration);

private:

    DeltaPoseReconstruction::DeltaPoseReconstructor m_reconstructor; /// reconstructor which provides delta poses between consecutive frames
    Calibration::CalibrationModule m_calibModule; ///< calibration module which stores a loaded calibration (and in future computes it)
    bool m_debugOutputActive; ///< indicates whether debug output is currently active

};

} //namespace Master
} //namespace VOCPP

#endif /* VOCPP_MASTER_H */
