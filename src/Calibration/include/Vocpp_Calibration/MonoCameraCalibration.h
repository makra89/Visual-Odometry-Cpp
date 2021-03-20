/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_MONO_CAMERA_CALIBRATION_H
#define VOCPP_MONO_CAMERA_CALIBRATION_H

#include <Vocpp_Interface/Types.h>
#include <opencv2/core/core.hpp>

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
      * /brief Default constructor for an invalid camera calibration
      */
    MonoCameraCalibration() : m_calibrationMatrix(cv::Mat1d::eye(3,3)),
        m_validCalib(false)
    {
    }
    
    /**
      * /brief Constructor specifying parameters for calibration matrix
      */
    MonoCameraCalibration(const double& in_focLength, const double& in_cameraCentX
        ,
        const double& in_cameraCentY, const double& in_skew) : m_validCalib(false)
    {
        m_calibrationMatrix = cv::Mat1d::zeros(3, 3);
        m_calibrationMatrix(0, 0) = in_focLength;
        m_calibrationMatrix(0, 1) = in_skew;
        m_calibrationMatrix(1, 1) = in_focLength;
        m_calibrationMatrix(0, 2) = in_cameraCentX;
        m_calibrationMatrix(1, 2) = in_cameraCentY;
        m_calibrationMatrix(2, 2) = 1.0F;

        m_validCalib = true;
    }

    MonoCameraCalibration& operator=(const MonoCameraCalibration& in_right)
    {
        this->m_calibrationMatrix = in_right.m_calibrationMatrix.clone();
        this->m_validCalib = in_right.m_validCalib;
        return *this;
    }

    MonoCameraCalibration(const MonoCameraCalibration& in_right)
    {
        this->m_calibrationMatrix = in_right.m_calibrationMatrix.clone();
        this->m_validCalib = in_right.m_validCalib;
    }


    cv::Mat1d GetCalibrationMatrix() const
    {
        return m_calibrationMatrix.clone();
    }

    /**
      * /brief Specifies whether the camera calibration is valid
      */
    bool IsValid() const
    {
        return m_validCalib;
    }

private:

    cv::Mat1d m_calibrationMatrix;
    bool m_validCalib;

};

} //namespace Calibration
} //namespace VOCPP

#endif /* VOCPP_MONO_CAMERA_CALIBRATION_H */
