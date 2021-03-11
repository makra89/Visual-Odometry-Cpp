/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#pragma once

#include <Vocpp_Utils/ConversionUtils.h>

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
cv::Mat1f GetWindowKernel(const int size);

/**
* /brief Convolve image with given kernel
*/
void ApplyKernelToImage(const cv::Mat1f& in_image, const cv::Mat1f& in_kernel, cv::Mat1f& out_image);

/**
* /brief Calculate intensity gradients of image in both x and y directions
*/
void Compute2DGradients(const cv::Mat1f& in_image, cv::Mat1f& out_gradX, cv::Mat1f& out_gradY);


struct LocalMaximum
{
    float posX;
    float posY;
    float value;

    bool operator > (const LocalMaximum& right) const
    {
        return value > right.value;
    }
};
/**
    * /brief Extract local maxima (with respect to intensity) of an image
    *
    * \param[in] in_image Image maxima shall be extracted from
    * \param[in] in_distance minimum distance between maxima [pixels]
    * \param[out] out_localMaxima vector of extracted local maxima
    * \param[in] in_subPixelCalculationDistance distance used for calculation of subPixel position of local maxima
*/
void ExtractLocalMaxima(const cv::Mat1f& in_image, const int in_distance, std::vector<LocalMaximum>& out_localMaxima);

/**
* /brief Extract patch of an image around a center position with a given distance
*
* \param[in] in_image Image from which the patch shall be extracted
* \param[out] out_patch image patch, has to be square matrix with dimension = in_distanceAroundCenter * 2 + 1
* \param[in] in_distanceAroundCenter distance to center that shall be included in the patch [pixels]
* \param[in] in_pixelPosX center location in X
* \param[in] in_pixelPosY center location in Y
*/
bool ExtractImagePatchAroundPixelPos(const cv::Mat1f& in_image, cv::Mat1f& out_patch, const int in_distanceAroundCenter, const int in_pixelPosX, const int in_pixelPosY);

/**
* /brief Normalize set out points with respect to their distance to the origin. Resulting point set will be centered at (0, 0)
and will have average distance of sqrt(2). Additionally the 3x3 transformation is provided which has been used to transform the
point set.
*/
void NormalizePointSet(const std::vector<cv::Point2f>& in_points, std::vector<cv::Point2f>& out_normPoints, cv::Mat1f& out_transform);

/**
* /brief Projection matrix using a rotation + translation + camera calibration matrix
* to a 3D world coordinate --> result will be the 2D image coordinates of the 3D point
*/
class ImageProjectionMatrix
{
public:
    ImageProjectionMatrix()
    {
        cv::hconcat(cv::Mat1f::eye(3, 3), cv::Mat1f::zeros(3, 1), m_projectMat);
    }

    ImageProjectionMatrix(const cv::Mat1f& in_rotationMatrix, const cv::Mat1f& in_translation,
        const cv::Mat1f& in_calibMatrix)
    {
        cv::hconcat(in_rotationMatrix, in_translation, m_projectMat);
        m_projectMat = in_calibMatrix * m_projectMat;
    }

    cv::Point2f Apply(cv::Point3f in_worldCoord)
    {
        cv::Mat1f homWorldCoord = Point3fToMatHomCoordinates(in_worldCoord);
        cv::Mat1f imageCoord = m_projectMat * homWorldCoord;
        cv::Point2f imageCoordOut;
        imageCoordOut.x = imageCoord(0, 0) / imageCoord(0, 2);
        imageCoordOut.y = imageCoord(1, 0) / imageCoord(0, 2);

        return imageCoordOut;
    }

    const cv::Mat1f& GetRawProjMat() const
    {
        return m_projectMat;
    }

private:
    cv::Mat1f m_projectMat;
};

/**
* /brief Projection matrix using a rotation + translation + camera calibration matrix
* to a 3D world coordinate --> result will be the coordinates of the 3D point in camera perspective
*/
class CameraProjectionMatrix
{
public:
    CameraProjectionMatrix(const cv::Mat1f& in_rotationMatrix, const cv::Mat1f& in_translation)
    {
        cv::hconcat(in_rotationMatrix, in_translation, m_projectMat);
        m_rotation = in_rotationMatrix;
        m_translation = in_translation;
    }

    cv::Point3f Apply(cv::Point3f in_worldCoord)
    {
        cv::Mat1f homWorldCoord = Point3fToMatHomCoordinates(in_worldCoord);
        cv::Mat1f cameraCoord = m_projectMat * homWorldCoord;
        cv::Point3f cameraCoordOut;
        cameraCoordOut.x = cameraCoord(0, 0);
        cameraCoordOut.y = cameraCoord(1, 0);
        cameraCoordOut.z = cameraCoord(2, 0);

        return cameraCoordOut;
    }

    ImageProjectionMatrix ConvertToImageProjectionMatrix(const cv::Mat1f& in_calibMatrix)
    {
        return ImageProjectionMatrix(m_rotation, m_translation, in_calibMatrix);
    }

    const cv::Mat1f& GetRotationMat() const
    {
        return m_rotation;
    }

    const cv::Mat1f& GetTranslation() const
    {
        return m_translation;
    }

private:
    cv::Mat1f m_projectMat;
    cv::Mat1f m_translation;
    cv::Mat1f m_rotation;
};

/**
* /brief Get cross product generating matrix from a vector
*/
void GetCrossProductMatrix(const cv::Vec3f& in_vec, cv::Mat1f& out_crossMat);

/**
* /brief Decompose an essential matrix into a translation vector and a rotation
*
* The provided points have to be in homogenous camera coordinates (Undo multiplication with calib mat).
* The terms "left" and "right" refer to the formulate x_left * E * x_right = 0
* \param[in] in_essentialMat essential matrix to be composed
* \param[in] in_calibMat calibration matrix
* \param[in] in_imageCoordLeft correspondence point in camera coordinates in left image
* \param[in] in_imageCoordRight correspondence point in camera coordinates in right image
* \param[in, out] inout_inlierMask gives inliers, 1 for inlier, 0 for outlier
* \param[out] out_translation translation from left camera center to right camera center in the left camera coordinate system
* \param[out] out_rotMatrix rotation matrix used to transfrom a point in right camera frame to left camera frame
* \param[out] out_triangulatedPoints 3D triangulated points
*/
bool DecomposeEssentialMatrix(const cv::Mat1f& in_essentialMat, const cv::Mat1f& in_calibMat, const std::vector<cv::Point2f>& in_imageCoordLeft,
    const std::vector <cv::Point2f>& in_imageCoordRight, std::vector<unsigned int>& inout_inlierIndices, cv::Mat1f& out_translation, cv::Mat1f& out_rotMatrix, std::vector<cv::Point3f>& out_triangulatedPoints);

/**
* /brief Triangulates a point in 3D given two camera coordinates and two projection matrices
*
*/
bool PointTriangulationLinear(const ImageProjectionMatrix& projMatLeft, const ImageProjectionMatrix& in_projMatRight, const cv::Point2f& in_imageCoordLeft,
    const cv::Point2f& in_imageCoordRight, cv::Point3f& out_triangulatedPoint);

} //namespace Utils
} //namespace VOCPP

