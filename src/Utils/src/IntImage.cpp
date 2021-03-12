/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/IntImage.h>
#include<iostream>
namespace VOCPP
{
namespace Utils
{

IntImage::IntImage(const cv::Mat1d& in_image) : m_intImage(cv::Mat1d::zeros(in_image.rows, in_image.cols))
{
    FillIntImage(in_image);
}

double IntImage::GetIntensity(const uint32_t& in_imgRow, const uint32_t& in_imgCol)
{
    return m_intImage.ptr<double>(in_imgRow)[in_imgCol];
}

bool IntImage::GetAreaAroundPixel(const uint32_t& in_centRow, const uint32_t& in_centCol, const uint32_t& in_distance, double& out_area)
{
    bool ret = false;
    
    bool inRangeRow = static_cast<int32_t>(in_centRow - in_distance) - 1 >= 0 && static_cast<int32_t>(in_centRow + in_distance) < m_intImage.rows;
    bool inRangeCol = static_cast<int32_t>(in_centCol - in_distance) - 1 >= 0 && static_cast<int32_t>(in_centCol + in_distance) < m_intImage.cols;

    if (inRangeCol && inRangeRow)
    {
        const double upperLeft = GetIntensity(in_centRow - in_distance - 1, in_centCol - in_distance - 1);
        const double upperRight = GetIntensity(in_centRow - in_distance - 1, in_centCol + in_distance);
        const double lowerLeft = GetIntensity(in_centRow + in_distance, in_centCol - in_distance - 1);
        const double lowerRight = GetIntensity(in_centRow + in_distance, in_centCol + in_distance);

        out_area = lowerRight - lowerLeft - upperRight + upperLeft;
        ret = true;
    }
    
    return ret;
}

void IntImage::FillIntImage(const cv::Mat1d& in_image)
{
    for (int32_t j = 0; j < in_image.rows; j++)
    {
        double* currRowPtrIntImage = m_intImage.ptr<double>(j);
        const double* currRowPtrImage = in_image.ptr<double>(j);
        for (int i = 0; i < in_image.cols; i++)
        {
            double area = 0;

            // Pixel left
            if (i - 1 >= 0)
            {
                area += currRowPtrIntImage[i - 1];
            }
            // Pixel above
            if (j - 1 >= 0)
            {   
                const double* upRowPtrIntImage = m_intImage.ptr<double>(j - 1);
                area += upRowPtrIntImage[i];
            }
            // Substract pixel that is counted twice
            if (i - 1 >= 0 && j - 1 >= 0)
            {
                const double* upRowPtrIntImage = m_intImage.ptr<double>(j - 1);
                area -= upRowPtrIntImage[i - 1];
            }
            // Add value of current pixel
            area += currRowPtrImage[i];
            // Fill value
            currRowPtrIntImage[i] = area;
        }
    }
}

} //namespace Utils
} //namespace VOCPP
