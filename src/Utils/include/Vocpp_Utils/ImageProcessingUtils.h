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
* /brief Provides window kernel of a given size
*
* \param[in] size of window kernel [pixels]
*/
cv::Mat GetWindowKernel(const uint32_t size);

/**
* /brief Convolve image with given kernel
*/
void ApplyKernelToImage(const cv::Mat& in_image, const cv::Mat& in_kernel, cv::Mat& out_image);

/**
* /brief Calculate intensity gradients of image in both x and y directions
*/
void Compute2DGradients(const cv::Mat& in_image, cv::Mat& out_gradX, cv::Mat& out_gradY);

/**
    * /brief Extract local maxima (with respect to intensity) of an image
    *
    * \param[in] in_image Image maxima shall be extracted from
    * \param[in] in_distance minimum distance between maxima [pixels]
    * \param[out] out_localMaxima vector of extracted local maxima
    * \param[in] in_subPixelCalculationDistance distance used for calculation of subPixel position of local maxima
*/
void ExtractLocalMaxima(const cv::Mat& in_image, const uint32_t in_distance, std::vector<cv::Point2f>& out_localMaxima, const uint32_t in_subPixelCalculationDistance = 0U);

/**
* /brief Extract patch of an image around a center position with a given distance
*
* \param[in] in_image Image from which the patch shall be extracted
* \param[out] out_patch image patch, has to be square matrix with dimension = in_distanceAroundCenter * 2 + 1
* \param[in] in_distanceAroundCenter distance to center that shall be included in the patch [pixels]
* \param[in] in_pixelPosX center location in X
* \param[in] in_pixelPosY center location in Y
*/
bool ExtractImagePatchAroundPixelPos(const cv::Mat& in_image, cv::Mat& out_patch, const uint32_t in_distanceAroundCenter, const uint32_t in_pixelPosX, const uint32_t in_pixelPosY);


/**
* /brief Get image coordinates of matching keypoints. Matching points have the same index in the output vector.
* Since in_matchesLeftRight might contain more than only the matches of left to right frame, an index in_imgIdx
* corresponding to cv::DMatch::imgIdx has to be provided.
*
* \param[in] in_left keypoints with indices corresponding to cv::DMatch::queryIdx of matching vector
* \param[in] in_right keypoints with indices corresponding to cv::DMatch::trainIdx of matching vector (extracted from frame with ID in_imgIdx)
* \param[in] in_matchesLeftRight vector of DMatches containing matching keypoints
* \param[in] in_imgIdx index of frame for which the matches shall be extracted from in_matchesLeftRight.
* \param[out] out_leftPoints image coordinates of left keypoint vector
* \param[out] out_rightPoints image coordinates of left keypoint vector
*/
void ComputeMatchingPoints(const std::vector<cv::KeyPoint>& in_left, const std::vector<cv::KeyPoint>& in_right, const std::vector<cv::DMatch>& in_matchesLeftRight,
    const int in_imgIdx, std::vector<cv::Point2f>& out_leftPoints, std::vector<cv::Point2f>& out_rightPoints);

/**
* /brief Normalize set out points with respect to their distance to the origin. Resulting point set will be centered at (0, 0)
and will have average distance of sqrt(2). Additionally the 3x3 transformation is provided which has been used to transform the
point set.
*/
void NormalizePointSet(const std::vector<cv::Point2f>& in_points, std::vector<cv::Point2f>& out_normPoints, cv::Mat& out_transform);

/**
* /brief Get 4x3 projection matrix using a rotation + translation
*/
bool GetProjectionMatrix(const cv::Mat& in_rotationMatrix, const cv::Mat& in_translation, cv::Mat& out_projMat);

/**
* /brief Get cross product generating matrix
*/
bool GetCrossProductMatrix(const cv::Mat& in_generatingMat, cv::Mat& out_crossMat);

} //namespace Utils
} //namespace VOCPP

