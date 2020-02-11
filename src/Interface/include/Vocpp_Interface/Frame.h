/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_FRAME_H
#define VOCPP_FRAME_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
namespace VOCPP
{

/**
  * /brief Invalid frame Id marker
  */
static int s_invalidFrameId = INT_MIN;

/**
  * /brief Checks whether a frame has a valid Id
  */
static bool IsValidFrameId(const int in_frameId)
{
    return in_frameId > s_invalidFrameId ? true : false;
}

/**
  * /brief Image wrapper class
  */
class Frame
{
public:

    /**
      * /brief Default constructor, will create invalid frame
      */
    Frame() :
        m_Id(s_invalidFrameId),
        m_validFrame(false)
    {
    }
    
    /**
      * /brief Constructor with grayscale(!) image data of type CV_32F.
      * No other type will currently be accepted
      *
      * \param[in] in_grayImage image will be moved to frame object !!!
      * \param[in] in_imgId ID of frame to be created
      */
    Frame(cv::Mat&& in_grayImage, const int in_imgId)
    {
        // No image data
        if (!in_grayImage.data)
        {
            std::cout << "[Frame]: No image data provided" << std::endl;
            m_validFrame = false;
        }
        // No grayscale image data
        else if (in_grayImage.type() != CV_32F)
        {
            std::cout << "[Frame]: Only images with type CV_32F are accepted" << std::endl;
            m_validFrame = false;
        }
        // Valid image data
        else
        {
            m_grayImage = std::move(in_grayImage);
            m_Id = in_imgId;
            m_validFrame = true;
        }
    }

    /**
      * /brief Get image data (const reference), either raw or grayscale
      */
    const cv::Mat& GetImage() const
    {
        return m_grayImage;
    }

    /**
      * /brief Get copy of image data, either raw or grayscale
      */
    cv::Mat GetImageCopy() const
    {
        return m_grayImage;
    }

    /**
      * /brief Returns frame ID
      */
    const int GetId() const
    {
        return m_Id;
    }

    /**
      * /brief Returns whether image frame data is valid
      */
    const bool isValid() const
    {
        return m_validFrame;
    }

private:

    cv::Mat m_grayImage; ///< grayscale image data

    int m_Id; ///< Id of the frame, must be a unique one!
    bool m_validFrame; ///< indicated whether this frame is a valid one

};

} //namespace VOCPP

#endif /* VOCPP_FRAME_H */
