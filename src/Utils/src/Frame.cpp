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

Frame::Frame(cv::Mat&& in_image, const uint32_t in_imgId)
{
	if (!in_image.data)
	{
		std::cout << "[Frame]: No image data provided" << std::endl;
		m_validFrame = false;
	}
	else
	{
		m_image = std::move(in_image);
		//convert to grayscale image
		cv::cvtColor(m_image, m_grayImage, cv::COLOR_BGR2GRAY);

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
