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
      * /brief Frame constructor using a pointer to the image data. Constructing a frame this way 
      * no memory is allocated. The frame is just a view on the data. The pointer has to be valid
      * for the whole lifetime of the frame object.
      */
    Frame(float* const in_grayImgData, int in_width, int in_height, int in_frameId) : m_Id(s_invalidFrameId)
    {
        // No image data
        if (in_grayImgData == NULL)
        {
            std::cout << "[Frame]: No image data provided" << std::endl;
            m_validFrame = false;
        }
        // Valid image data
        else
        {
            m_grayImage = cv::Mat_<float>(in_height, in_width, in_grayImgData);
            m_Id = in_frameId;
            m_validFrame = true;
        }
    }


    /**
      * /brief Get image data (const reference), either raw or grayscale
      */
    const cv::Mat_<float>& GetImage() const
    {
        return m_grayImage;
    }

    /**
      * /brief Get copy of image data, either raw or grayscale
      */
    cv::Mat_<float> GetImageCopy() const
    {
        return m_grayImage.clone();
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
    const bool IsValid() const
    {
        return m_validFrame;
    }

private:

    cv::Mat_<float> m_grayImage; ///< grayscale image data

    int m_Id; ///< Id of the frame, must be a unique one!
    bool m_validFrame; ///< indicated whether this frame is a valid one

};

} //namespace VOCPP

#endif /* VOCPP_FRAME_H */
