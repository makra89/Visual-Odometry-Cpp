/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/Frame.h>
#include<opencv2/imgproc.hpp>
#include<iostream>

namespace VOCPP
{
namespace Utils
{

Frame::Frame()
{
    m_validFrame = false;
}

Frame::Frame(cv::Mat&& in_imageGray, const uint32_t in_imgId)
{
    // No image data
    if (!in_imageGray.data)
    {
        std::cout << "[Frame]: No image data provided" << std::endl;
        m_validFrame = false;
    }
    // No grayscale image data
    else if (in_imageGray.type() != CV_32F)
    {
        std::cout << "[Frame]: Only images with type CV_32F are accepted" << std::endl;
        m_validFrame = false;
    }
    // Valid image data
    else
    {
        m_grayImage = std::move(in_imageGray);
        m_Id = in_imgId;
        m_validFrame = true;
    }
}

void Frame::SetKeypoints(std::vector<cv::KeyPoint>&& in_keypoints)
{
    m_keypoints = std::move(in_keypoints);
}

void Frame::SetDescriptions(std::vector<cv::Mat>&& in_descriptions)
{
    m_descriptions = std::move(in_descriptions);
}

void Frame::SetMatches(std::vector<cv::DMatch>&& in_matches)
{
    m_matches = std::move(in_matches);
}

} //namespace Utils
} //namespace VOCPP
