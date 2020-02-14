/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Calibration/CalibrationModule.h>
#include<iostream>

namespace VOCPP
{
namespace Calibration
{

CalibrationModule::CalibrationModule() : m_monoCalib(cv::Mat::eye(3, 3, CV_32F))
{
}

CalibrationModule::~CalibrationModule()
{
}

} //namespace Calibration
} //namespace VOCPP
