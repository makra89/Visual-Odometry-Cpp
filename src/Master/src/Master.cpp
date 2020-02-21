/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Master/Master.h>
#include<iostream>
#include <opencv2/opencv.hpp>



namespace VOCPP
{
namespace Master
{

Master::Master() : m_reconstructor(), m_calibModule()
{
}

Master::~Master()
{
}

bool Master::FeedNextFrame(const Frame& in_frame)
{  
    // Try to get a valid mono camera calibration from the calibration module
    Calibration::MonoCameraCalibration monoCalib;
    bool ret = m_calibModule.GetSavedMonoCalib(monoCalib);

    // Only if a valid calibration is there we can feed the frame to the reconstructor
    ret = ret && m_reconstructor.FeedNextFrame(in_frame, monoCalib.GetCalibrationMatrix());
    return ret;
}

DeltaCameraPose Master::GetLastDeltaPose()
{
    return m_reconstructor.GetLastDeltaPose();
}

CameraPose Master::GetLastPose()
{
    return m_reconstructor.GetLastPose();
}

bool Master::LoadCalibration(const Calibration::MonoCameraCalibration& in_monoCalibration)
{
    return m_calibModule.LoadCalibration(in_monoCalibration);
}

} //namespace Master
} //namespace VOCPP
