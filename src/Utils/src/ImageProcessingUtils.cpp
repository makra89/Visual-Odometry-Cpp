/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <Vocpp_Utils/ConversionUtils.h>
#include<opencv2/imgproc.hpp>
#include<iostream>

namespace VOCPP
{
namespace Utils
{

cv::Mat1f GetWindowKernel(const int size)
{
    return cv::Mat1f::ones(size, size);
}

void ApplyKernelToImage(const cv::Mat1f& in_image, const cv::Mat1f& in_kernel, cv::Mat1f& out_image)
{
    cv::filter2D(in_image, out_image, CV_32F, in_kernel);
}

void Compute2DGradients(const cv::Mat1f& in_image, cv::Mat1f& out_gradX, cv::Mat1f& out_gradY)
{ 
    // Compute gradient using sobel operators
    // Sobel operator can be replaced by two convolutions
    float dataRowX[3] = { -1.0, 0.0, 1.0 };
    cv::Mat1f sobelRowX = cv::Mat1f(1, 3, dataRowX);
    float dataColX[3] = { 1.0, 2.0, 1.0 };
    cv::Mat1f sobelColX = cv::Mat1f(3, 1, dataColX);
    cv::sepFilter2D(in_image, out_gradX, CV_32F, sobelRowX, sobelColX);

    float dataRowY[3] = { 1.0, 2.0, 1.0 };
    cv::Mat1f sobelRowY = cv::Mat1f(1, 3, dataRowY);
    float dataColY[3] = { -1.0, 0.0, 1.0 };
    cv::Mat1f sobelColY = cv::Mat1f(3, 1, dataColY);
    cv::sepFilter2D(in_image, out_gradY, CV_32F, sobelRowY, sobelColY);
}

void ExtractLocalMaxima(const cv::Mat1f& in_image, const int in_distance, std::vector<LocalMaximum>& out_localMaxima)
{
    cv::Mat1f dilatedImage;
    cv::Mat1f kernel = GetWindowKernel(2 * in_distance + 1);
    cv::dilate(in_image, dilatedImage, kernel);
    
    for (int i = 0; i < in_image.size[0]; i++)
    {
        const float* rowImage = in_image.ptr<float>(i);
        const float* rowDilImage = dilatedImage.ptr<float>(i);
        for (int j = 0; j < in_image.size[1]; j++)
        {
            bool greaterZero = rowImage[j] > 0.0;
            // If image value is equal to the dilated value (maximum value of surrounding pixels)
            // this pixel has the maximum value
            if (greaterZero && rowImage[j] == rowDilImage[j])
            {
                out_localMaxima.push_back(LocalMaximum{ static_cast<float>(j), static_cast<float>(i),rowImage[j]});
            }
        }
    }
}

bool ExtractImagePatchAroundPixelPos(const cv::Mat1f& in_image, cv::Mat1f& out_patch, const int in_distanceAroundCenter, const int in_pixelPosX, const int in_pixelPosY)
{
    bool ret = true;

    // First check whether all coordinates are within range of in_image
    const bool inRangeX = static_cast<int>(in_pixelPosX - in_distanceAroundCenter) >= 0 && static_cast<int>(in_pixelPosX + in_distanceAroundCenter) < in_image.size[1];
    const bool inRangeY = static_cast<int>(in_pixelPosY - in_distanceAroundCenter) >= 0 && static_cast<int>(in_pixelPosY + in_distanceAroundCenter) < in_image.size[0];
    // Then check if dimensions of provided patch matrix are correct
    const bool correctPatchDimX = out_patch.size[1] == (2 * in_distanceAroundCenter + 1);
    const bool correctPatchDimY = out_patch.size[0] == (2 * in_distanceAroundCenter + 1);

    if (inRangeX && inRangeY && correctPatchDimX && correctPatchDimY)
    {
        for (int j = 0; j < out_patch.size[0]; j++)
        {
            int imagePosY = in_pixelPosY - in_distanceAroundCenter + j;
            const float* rowPtr = in_image.ptr<float>(imagePosY);
            float* rowPtrPatch = out_patch.ptr<float>(j);
            for (int i = 0; i < out_patch.size[1]; i++)
            {
                int imagePosX = in_pixelPosX - in_distanceAroundCenter + i;
                rowPtrPatch[i] = rowPtr[imagePosX];
            }
        }
    }
    else if (!correctPatchDimX || !correctPatchDimY)
    {
        ret = false;
        std::cout << "[ExtractImagePatchAroundPixelPos]: The dimension of the patch matrix are wrong" << std::endl;
    }
    else
    {
        ret = false;
    }

    return ret;

}

void NormalizePointSet(const std::vector<cv::Point2f>& in_points, std::vector<cv::Point2f>& out_normPoints, cv::Mat1f& out_transform)
{
    float meanX = 0, meanY = 0;
    for (auto element : in_points)
    {
        meanX += static_cast<float>(element.x / in_points.size());
        meanY += static_cast<float>(element.y / in_points.size());
    }

    float meanDist = 0;
    for (auto element : in_points)
    {
        const float shiftedX = static_cast<float>(element.x) - meanX;
        const float shiftedY = static_cast<float>(element.y) - meanY;

        meanDist += ((std::pow(shiftedX, 2) + std::pow(shiftedY, 2)) / in_points.size());
    }

    const float scale = std::sqrt(2.0F) / std::sqrt(meanDist);

    for (auto element : in_points)
    {
        const float normX = scale * (static_cast<float>(element.x) - meanX);
        const float normY = scale * (static_cast<float>(element.y) - meanY);

        out_normPoints.push_back(cv::Point2f(normX, normY));
    }

    out_transform = cv::Mat1f::zeros(3, 3);
    out_transform(0, 0) = scale;
    out_transform(1, 1) = scale;
    out_transform(0, 2) = -scale * meanX;
    out_transform(1, 2) = -scale * meanY;
    out_transform(2, 2) = 1.0;
}

void GetCrossProductMatrix(const cv::Vec3f& in_vec, cv::Mat1f& out_crossMat)
{
    out_crossMat = cv::Mat1f::zeros(3, 3);
    out_crossMat(0, 1) = -in_vec[2];
    out_crossMat(0, 2) = in_vec[1];
    out_crossMat(1, 0) = in_vec[2];
    out_crossMat(1, 2) = -in_vec[0];
    out_crossMat(2, 0) = -in_vec[1];
    out_crossMat(2, 1) = in_vec[0];
}

bool DecomposeEssentialMatrix(const cv::Mat1f& in_essentialMat, const cv::Mat1f& in_calibMat, const cv::Point2f& in_imageCoordLeft,
    const cv::Point2f& in_imageCoordRight, cv::Mat1f& out_translation, cv::Mat1f& out_rotMatrix)
{
    // Compute SVD
    cv::Mat1d U, D, V_t;
    cv::Mat1d essentialMatd;
    in_essentialMat.convertTo(essentialMatd, CV_64F);
    cv::SVDecomp(essentialMatd, D, U, V_t, cv::SVD::FULL_UV);
   
    // Set smallest singular value to zero and recompute SVD
    cv::Mat1d newDiag = cv::Mat1d::zeros(3, 3);
    newDiag(0, 0) = 1.0;
    newDiag(1, 1) = 1.0;

    cv::Mat1d newEssentialMat = U * (newDiag * V_t);
    cv::SVDecomp(newEssentialMat, D, U, V_t, cv::SVD::FULL_UV);
    if (cv::determinant(U * V_t) < 0.0)
    {
        V_t = -V_t;
    }

    cv::Mat1d Y = cv::Mat1d::zeros(3, 3);
    Y(0, 1) = -1.0;
    Y(1, 0) = 1.0;
    Y(2, 2) = 1.0;

    // Two solutions for translation
    cv::Mat1d t1 = U.col(2);
    cv::Mat1d t2 = -t1;

    // Two solutions for rotation matrix
    cv::Mat1d R1 = U * (Y * V_t);
    cv::Mat1d R2 = U * (Y.t() * V_t);

    cv::Mat1f t1f;
    t1.convertTo(t1f, CV_32F);
    cv::Mat1f t2f;
    t2.convertTo(t2f, CV_32F);
    cv::Mat1f R1f;
    R1.convertTo(R1f, CV_32F);
    cv::Mat1f R2f;
    R2.convertTo(R2f, CV_32F);

    // Right projection matrix is trivial one
    CameraProjectionMatrix camProjMatRight(cv::Mat1f::eye(3, 3), cv::Mat1f::zeros(3, 1));

    // Four different projection matrices for right one
    std::vector<CameraProjectionMatrix> camProjectionMatCandidates;

    camProjectionMatCandidates.push_back(CameraProjectionMatrix(R1f, t1f));
    camProjectionMatCandidates.push_back(CameraProjectionMatrix(R1f, t2f));
    camProjectionMatCandidates.push_back(CameraProjectionMatrix(R2f, t1f));
    camProjectionMatCandidates.push_back(CameraProjectionMatrix(R2f, t2f));

    // Check which of the solutions is the correct one by demanding that the triangulated point is in front of both cameras
    for (auto candidate : camProjectionMatCandidates)
    {
        cv::Point3f triangPoint;
        PointTriangulationLinear(candidate.ConvertToImageProjectionMatrix(in_calibMat), camProjMatRight.ConvertToImageProjectionMatrix(in_calibMat),
            in_imageCoordLeft, in_imageCoordRight, triangPoint);
        cv::Point3f projPointLeft = candidate.Apply(triangPoint);
        cv::Point3f projPointRight = camProjMatRight.Apply(triangPoint);

        if (projPointLeft.z > 0.0 && projPointRight.z > 0)
        {
            out_rotMatrix = candidate.GetRotationMat();
            out_translation = candidate.GetTranslation();
        }
    }
  

    return false;
}

bool PointTriangulationLinear(const ImageProjectionMatrix& in_projMatLeft, const ImageProjectionMatrix& in_projMatRight, const cv::Point2f& in_imageCoordLeft,
    const cv::Point2f& in_imageCoordRight, cv::Point3f& out_triangulatedPoint)
{
    cv::Mat1f A = cv::Mat1f::zeros(4, 4);

    // Construct matrix to decompose (see for example https://www.uio.no/studier/emner/matnat/its/nedlagte-emner/UNIK4690/v16/forelesninger/lecture_7_2-triangulation.pdf)
    A(0, 0) = in_imageCoordLeft.y * in_projMatLeft.GetRawProjMat()(2, 0) - in_projMatLeft.GetRawProjMat()(1, 0);
    A(0, 1) = in_imageCoordLeft.y * in_projMatLeft.GetRawProjMat()(2, 1) - in_projMatLeft.GetRawProjMat()(1, 1);
    A(0, 2) = in_imageCoordLeft.y * in_projMatLeft.GetRawProjMat()(2, 2) - in_projMatLeft.GetRawProjMat()(1, 2);
    A(0, 3) = in_imageCoordLeft.y * in_projMatLeft.GetRawProjMat()(2, 3) - in_projMatLeft.GetRawProjMat()(1, 3);

    A(1, 0) = in_imageCoordLeft.x * in_projMatLeft.GetRawProjMat()(2, 0) - in_projMatLeft.GetRawProjMat()(0, 0);
    A(1, 1) = in_imageCoordLeft.x * in_projMatLeft.GetRawProjMat()(2, 1) - in_projMatLeft.GetRawProjMat()(0, 1);
    A(1, 2) = in_imageCoordLeft.x * in_projMatLeft.GetRawProjMat()(2, 2) - in_projMatLeft.GetRawProjMat()(0, 2);
    A(1, 3) = in_imageCoordLeft.x * in_projMatLeft.GetRawProjMat()(2, 3) - in_projMatLeft.GetRawProjMat()(0, 3);

    A(2, 0) = in_imageCoordRight.y * in_projMatRight.GetRawProjMat()(2, 0) - in_projMatRight.GetRawProjMat()(1, 0);
    A(2, 1) = in_imageCoordRight.y * in_projMatRight.GetRawProjMat()(2, 1) - in_projMatRight.GetRawProjMat()(1, 1);
    A(2, 2) = in_imageCoordRight.y * in_projMatRight.GetRawProjMat()(2, 2) - in_projMatRight.GetRawProjMat()(1, 2);
    A(2, 3) = in_imageCoordRight.y * in_projMatRight.GetRawProjMat()(2, 3) - in_projMatRight.GetRawProjMat()(1, 3);

    A(3, 0) = in_imageCoordRight.x * in_projMatRight.GetRawProjMat()(2, 0) - in_projMatRight.GetRawProjMat()(0, 0);
    A(3, 1) = in_imageCoordRight.x * in_projMatRight.GetRawProjMat()(2, 1) - in_projMatRight.GetRawProjMat()(0, 1);
    A(3, 2) = in_imageCoordRight.x * in_projMatRight.GetRawProjMat()(2, 2) - in_projMatRight.GetRawProjMat()(0, 2);
    A(3, 3) = in_imageCoordRight.x * in_projMatRight.GetRawProjMat()(2, 3) - in_projMatRight.GetRawProjMat()(0, 3);

    // Compute SVD
    cv::Mat1f U, D, V_t;
    cv::SVDecomp(A, D, U, V_t, cv::SVD::FULL_UV);

    // Solution is given by nullspace of A
    out_triangulatedPoint.x = V_t.row(3)(0, 0) / V_t.row(3)(0, 3);
    out_triangulatedPoint.y = V_t.row(3)(0, 1) / V_t.row(3)(0, 3);
    out_triangulatedPoint.z = V_t.row(3)(0, 2) / V_t.row(3)(0, 3);

    return true;
}


} //namespace Utils
} //namespace VOCPP
