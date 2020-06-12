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

IntImage::IntImage(const cv::Mat1f& in_image) : m_intImage(cv::Mat1f::zeros(in_image.rows, in_image.cols))
{
    FillIntImage(in_image);
}

float IntImage::GetIntensity(const int& in_imgRow, const int& in_imgCol)
{
    return m_intImage.ptr<float>(in_imgRow)[in_imgCol];
}

bool IntImage::GetAreaAroundPixel(const int& in_centRow, const int& in_centCol, const int& in_distance, float& out_area)
{
    bool ret = false;
    
    bool inRangeRow = in_centRow - in_distance - 1 >= 0 && in_centRow + in_distance < m_intImage.rows;
    bool inRangeCol = in_centCol - in_distance - 1 >= 0 && in_centCol + in_distance < m_intImage.cols;

    if (inRangeCol && inRangeRow)
    {
        const float upperLeft = GetIntensity(in_centRow - in_distance - 1, in_centCol - in_distance - 1);
        const float upperRight = GetIntensity(in_centRow - in_distance - 1, in_centCol + in_distance);
        const float lowerLeft = GetIntensity(in_centRow + in_distance, in_centCol - in_distance - 1);
        const float lowerRight = GetIntensity(in_centRow + in_distance, in_centCol + in_distance);

        out_area = lowerRight - lowerLeft - upperRight + upperLeft;
        ret = true;
    }
    
    return ret;
}

void IntImage::FillIntImage(const cv::Mat1f& in_image)
{
    for (int j = 0; j < in_image.rows; j++)
    {
        float* currRowPtrIntImage = m_intImage.ptr<float>(j);
        const float* currRowPtrImage = in_image.ptr<float>(j);
        for (int i = 0; i < in_image.cols; i++)
        {
            float area = 0;

            // Pixel left
            if (i - 1 >= 0)
            {
                area += currRowPtrIntImage[i - 1];
            }
            // Pixel above
            if (j - 1 >= 0)
            {   
                const float* upRowPtrIntImage = m_intImage.ptr<float>(j - 1);
                area += upRowPtrIntImage[i];
            }
            // Substract pixel that is counted twice
            if (i - 1 >= 0 && j - 1 >= 0)
            {
                const float* upRowPtrIntImage = m_intImage.ptr<float>(j - 1);
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
