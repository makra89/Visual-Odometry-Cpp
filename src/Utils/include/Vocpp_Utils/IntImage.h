/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_INTEGRAL_IMAGE_H
#define VOCPP_INTEGRAL_IMAGE_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <map>

namespace VOCPP
{
namespace Utils
{

/**
 *  \brief Integral image implementation class
 * Can be used to calculate areas on an image very efficiently
 */
class IntImage
{
public:

    /**
      * \brief Constructor
      */
    IntImage(const cv::Mat1f& in_image);

    /**
      * \brief Intensity getter for a certain pixel
      * 
      * \return Area enclosed in the rectangle (0,0), (0,in_imgCol), (in_imgRow,0), and (in_imgRow,in_imgCol)
      */
    float GetIntensity(const int& in_imgRow, const int& in_imgCol);

    /**
      * \brief Get Intensity enclosed by the square defined by the center pixel and the radius (in_distance) around it
      * The edge pixel values are included! For example, when in_distance = 3 --> area of a 7x7 image patch is returned
      */
    bool GetAreaAroundPixel(const int& in_centRow, const int& in_centCol, const int& in_distance, float& out_area);

private:
    /**
      * \brief Internally used to fill the integral image in the constructor
      */
    void FillIntImage(const cv::Mat1f& in_image);

    cv::Mat1f m_intImage; ///< integral image values

};

} //namespace Utils
} //namespace VOCPP

#endif /* VOCPP_INTEGRAL_IMAGE_H */
