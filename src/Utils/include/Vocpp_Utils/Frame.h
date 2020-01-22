/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>

namespace VOCPP
{
namespace Utils
{

/**
  * /brief Image wrapper class
  */
class Frame
{
public:

    /**
      * /brief Default constructor, will create invalid frame
      */
    Frame();
    
    /**
      * /brief Constructor with grayscale(!) image data of type CV_32F.
      * No other type will currently be accepted
      *
      * \param[in] in_grayImage image will be moved to frame object !!!
      * \param[in] in_imgId ID of frame to be created
      */
    Frame(cv::Mat&& in_grayImage, const uint32_t in_imgId);

    /**
      * /brief Keypoint setter, input argument will be moved
      */
    void SetKeypoints(std::vector<cv::KeyPoint>&& in_keypoints);
    
    /**
      * /brief Description setter, input argument will be moved
      */
    void SetDescriptions(std::vector<cv::Mat>&& in_descriptions);
    
    /**
      * /brief Match setter, input argument will be moved
      */
    void SetMatches(std::vector<cv::DMatch>&& in_matches);

    /**
      * /brief Get image data (const reference), either raw or grayscale
      */
    const cv::Mat& GetImage()
    {
        return m_grayImage;
    }

    /**
      * /brief Get copy of image data, either raw or grayscale
      */
    cv::Mat GetImageCopy()
    {
        return m_grayImage;
    }

    /**
      * /brief Keypoint getter (const reference)
      */
    const std::vector<cv::KeyPoint>& GetKeypoints() const
    {
        return m_keypoints;
    }

    /**
      * /brief Keypoint getter (copy)
      */
    std::vector<cv::KeyPoint> GetKeypoints()
    {
        return m_keypoints;
    }

    /**
      * /brief Description getter
      */
    const std::vector<cv::Mat>& GetDescriptions() const
    {
        return m_descriptions;
    }

    /**
      * /brief Matches getter
      */
    const std::vector<cv::DMatch>& GetMatches() const
    {
        return m_matches;
    }

    /**
      * /brief Returns whether image frame data is valid
      */
    const bool isValid() const
    {
        return m_validFrame;
    }

    /**
      * /brief Returns frame ID
      */
    const uint32_t GetId() const
    {
        return m_Id;
    }


private:

    cv::Mat m_grayImage; ///< grayscale image data

    std::vector<cv::KeyPoint> m_keypoints;
    std::vector<cv::Mat> m_descriptions;
    std::vector<cv::DMatch> m_matches;

    uint32_t m_Id;
    bool m_validFrame; ///< indicated whether this frame is a valid one

};

} //namespace Utils
} //namespace VOCPP

