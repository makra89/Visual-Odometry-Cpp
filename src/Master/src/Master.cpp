/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Master/Master.h>
#include<iostream>



namespace VOCPP
{
namespace Master
{

Master::Master() : m_reconstructor(), m_calibModule()
{
}

bool Master::FeedNextFrame(const Utils::Frame& in_frame)
{  

    bool ret = m_reconstructor.FeedNextFrame(in_frame, m_calibModule.GetSavedMonoCalib().GetCalibrationMatrix());

    return ret;
}

bool Master::LoadCalibration(const Calibration::MonoCameraCalibration& in_monoCalibration)
{
    return m_calibModule.LoadCalibration(in_monoCalibration);
}

} //namespace Master
} //namespace VOCPP
