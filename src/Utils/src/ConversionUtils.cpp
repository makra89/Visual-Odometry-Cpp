/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/ConversionUtils.h>

namespace VOCPP
{
namespace Utils
{
    
cv::Mat1d Point2dToMatHomCoordinates(const cv::Point2d& in_point)
{
    cv::Mat1d ret = (cv::Mat1d(3, 1) << in_point.x, in_point.y, 1.0);

    return ret;
}

cv::Mat1d Point3dToMatHomCoordinates(const cv::Point3d& in_point)
{
    cv::Mat1d ret = (cv::Mat1d(4, 1) << in_point.x, in_point.y, in_point.z, 1.0);

    return ret;
}

} //namespace Utils
} //namespace VOCPP
