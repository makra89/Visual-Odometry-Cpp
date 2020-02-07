/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_MONO_CAMERA_CALIBRATION_H
#define VOCPP_MONO_CAMERA_CALIBRATION_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

namespace VOCPP
{
namespace Calibration
{
   
/**
  * /brief This class saves a camera calibration for a monocular camera
  */
class MonoCameraCalibration
{

public:
    
    /**
      * /brief Constructor
      */
    explicit MonoCameraCalibration(const cv::Mat& in_calibrationMatrix) : m_validCalib(false)
    {
        if (in_calibrationMatrix.type() != CV_32F)
        {
            std::cout << "[MonoCameraCalibration]: Invalid calibration matrix provided, must be of type CV_32F" << std::endl;
        }
        else if (in_calibrationMatrix.rows != 3 || in_calibrationMatrix.cols != 3)
        {
            std::cout << "[MonoCameraCalibration]: Invalid calibration matrix provided, must be 3x3" << std::endl;

        }
        else
        {
            m_calibrationMatrix = in_calibrationMatrix;
            m_validCalib = true;
        }
    }

    cv::Mat GetCalibrationMatrix() const
    {
        return m_calibrationMatrix;
    }

    /**
      * /brief Specifies whether the camera calibration is valid
      */
    bool IsValid() const
    {
        return m_validCalib;
    }

private:

    cv::Mat m_calibrationMatrix;
    bool m_validCalib;

};

} //namespace Calibration
} //namespace VOCPP

#endif /* VOCPP_MONO_CAMERA_CALIBRATION_H */
