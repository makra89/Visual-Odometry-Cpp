/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <Vocpp_Utils/ConversionUtils.h>
#include <Vocpp_Utils/TracingImpl.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace VOCPP
{
namespace Utils
{

cv::Mat1d GetWindowKernel(const int32_t size)
{
    return cv::Mat1d::ones(size, size);
}

void ApplyKernelToImage(const cv::Mat1d& in_image, const cv::Mat1d& in_kernel, cv::Mat1d& out_image)
{
    cv::filter2D(in_image, out_image, CV_64F, in_kernel);
}

void Compute2DGradients(const cv::Mat1d& in_image, cv::Mat1d& out_gradX, cv::Mat1d& out_gradY)
{ 
    // Compute gradient using sobel operators
    // Sobel operator can be replaced by two convolutions
    double dataRowX[3] = { -1.0, 0.0, 1.0 };
    cv::Mat1d sobelRowX = cv::Mat1d(1, 3, dataRowX);
    double dataColX[3] = { 1.0, 2.0, 1.0 };
    cv::Mat1d sobelColX = cv::Mat1d(3, 1, dataColX);
    cv::sepFilter2D(in_image, out_gradX, CV_64F, sobelRowX, sobelColX);

    double dataRowY[3] = { 1.0, 2.0, 1.0 };
    cv::Mat1d sobelRowY = cv::Mat1d(1, 3, dataRowY);
    double dataColY[3] = { -1.0, 0.0, 1.0 };
    cv::Mat1d sobelColY = cv::Mat1d(3, 1, dataColY);
    cv::sepFilter2D(in_image, out_gradY, CV_64F, sobelRowY, sobelColY);
}

void ExtractLocalMaxima(const cv::Mat1d& in_image, const int32_t in_distance, std::vector<LocalMaximum>& out_localMaxima)
{
    cv::Mat1d dilatedImage;
    uint32_t kernelSize = 2U * in_distance + 1U;
    cv::Mat1b kernel = cv::Mat1b::ones(kernelSize, kernelSize);
    kernel(in_distance, in_distance) = 0U;

    cv::dilate(in_image, dilatedImage, kernel);
    
    for (int32_t i = 0; i < in_image.size[0]; i++)
    {
        const double* rowImage = in_image.ptr<double>(i);
        const double* rowDilImage = dilatedImage.ptr<double>(i);
        for (int32_t j = 0; j < in_image.size[1]; j++)
        {
            bool greaterZero = rowImage[j] > 0.0;
            // If image value is equal to the dilated value (maximum value of surrounding pixels)
            // this pixel has the maximum value
            if (greaterZero && rowImage[j] > rowDilImage[j])
            {
                LocalMaximum localMax = { static_cast<double>(j), static_cast<double>(i), rowImage[j] };
                out_localMaxima.push_back(localMax);
            }
        }
    }
}

bool ExtractImagePatchAroundPixelPos(const cv::Mat1d& in_image, cv::Mat1d& out_patch, const uint32_t in_distanceAroundCenter, const uint32_t in_pixelPosX, const uint32_t in_pixelPosY)
{
    bool ret = true;

    // First check whether all coordinates are within range of in_image
    const bool inRangeX = static_cast<int32_t>(in_pixelPosX - in_distanceAroundCenter) >= 0U && static_cast<int32_t>(in_pixelPosX + in_distanceAroundCenter) < in_image.size[1];
    const bool inRangeY = static_cast<int32_t>(in_pixelPosY - in_distanceAroundCenter) >= 0U && static_cast<int32_t>(in_pixelPosY + in_distanceAroundCenter) < in_image.size[0];
    // Then check if dimensions of provided patch matrix are correct
    const bool correctPatchDimX = out_patch.size[1] == (2 * in_distanceAroundCenter + 1);
    const bool correctPatchDimY = out_patch.size[0] == (2 * in_distanceAroundCenter + 1);

    if (inRangeX && inRangeY && correctPatchDimX && correctPatchDimY)
    {
        for (int32_t j = 0; j < out_patch.size[0]; j++)
        {
            int32_t imagePosY = in_pixelPosY - in_distanceAroundCenter + j;
            const double* rowPtr = in_image.ptr<double>(imagePosY);
            double* rowPtrPatch = out_patch.ptr<double>(j);
            for (int32_t i = 0; i < out_patch.size[1]; i++)
            {
                int32_t imagePosX = in_pixelPosX - in_distanceAroundCenter + i;
                rowPtrPatch[i] = rowPtr[imagePosX];
            }
        }
    }
    else if (!correctPatchDimX || !correctPatchDimY)
    {
        ret = false;
        VOCPP_TRACE_WARNING("[ExtractImagePatchAroundPixelPos]: The dimension of the patch matrix are wrong")
    }
    else
    {
        ret = false;
    }

    return ret;

}

void NormalizePointSet(const std::vector<cv::Point2d>& in_points, std::vector<cv::Point2d>& out_normPoints, cv::Mat1d& out_transform)
{
    double meanX = 0, meanY = 0;
    for (uint32_t idx = 0U; idx < in_points.size(); idx++)
    {
        meanX += (in_points[idx].x / static_cast<double>(in_points.size()));
        meanY += (in_points[idx].y / static_cast<double>(in_points.size()));
    }

    double meanDist = 0;
    for (uint32_t idx = 0U; idx < in_points.size(); idx++)
    {
        const double shiftedX = in_points[idx].x - meanX;
        const double shiftedY = in_points[idx].y - meanY;

        meanDist += ((std::pow(shiftedX, 2) + std::pow(shiftedY, 2)) / in_points.size());
    }

    const double scale = std::sqrt(2.0F) / std::sqrt(meanDist);

    for (uint32_t idx = 0U; idx < in_points.size(); idx++)
    {
        const double normX = scale * (in_points[idx].x - meanX);
        const double normY = scale * (in_points[idx].y - meanY);

        out_normPoints.push_back(cv::Point2d(normX, normY));
    }

    out_transform = cv::Mat1d::zeros(3, 3);
    out_transform(0, 0) = scale;
    out_transform(1, 1) = scale;
    out_transform(0, 2) = -scale * meanX;
    out_transform(1, 2) = -scale * meanY;
    out_transform(2, 2) = 1.0;
}

void GetCrossProductMatrix(const cv::Vec3d& in_vec, cv::Mat1d& out_crossMat)
{
    out_crossMat = cv::Mat1d::zeros(3, 3);
    out_crossMat(0, 1) = -in_vec[2];
    out_crossMat(0, 2) = in_vec[1];
    out_crossMat(1, 0) = in_vec[2];
    out_crossMat(1, 2) = -in_vec[0];
    out_crossMat(2, 0) = -in_vec[1];
    out_crossMat(2, 1) = in_vec[0];
}

bool DecomposeEssentialMatrix(const cv::Mat1d& in_essentialMat, const cv::Mat1d& in_calibMat, const std::vector<cv::Point2d>& in_imageCoordLeft,
    const std::vector <cv::Point2d>& in_imageCoordRight, std::vector<uint32_t>& inout_inlierIndices, cv::Mat1d& out_translation,
    cv::Mat1d& out_rotMatrix, std::vector<cv::Point3d>& out_triangulatedPoints)
{
    // Compute SVD
    cv::Mat1d U, D, V_t;
    cv::SVDecomp(in_essentialMat, D, U, V_t, cv::SVD::FULL_UV);
   
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

    // Right projection matrix is trivial one
    CameraProjectionMatrix camProjMatRight(cv::Mat1d::eye(3, 3), cv::Mat1d::zeros(3, 1));

    // Four different projection matrices for right one
    std::vector<CameraProjectionMatrix> camProjectionMatCandidates;

    camProjectionMatCandidates.push_back(CameraProjectionMatrix(R1, t1));
    camProjectionMatCandidates.push_back(CameraProjectionMatrix(R1, t2));
    camProjectionMatCandidates.push_back(CameraProjectionMatrix(R2, t1));
    camProjectionMatCandidates.push_back(CameraProjectionMatrix(R2, t2));

    bool ret = false;

    // Test for at least 10 coordinates which projection matrix to choose
    const uint32_t numTestCandidates = std::min(10U, static_cast<uint32_t>(inout_inlierIndices.size()));
    uint32_t numVotes[4] = { 0U, 0U, 0U, 0U };
    
    for (uint32_t it = 0U; it < numTestCandidates; it++)
    {
        // Check which of the solutions is the correct one by demanding that the triangulated point is in front of both cameras
        for (uint32_t candidateId = 0U; candidateId < 4U; candidateId++)
        {
            cv::Point3d triangPoint;
            PointTriangulationLinear(camProjectionMatCandidates[candidateId].ConvertToImageProjectionMatrix(in_calibMat), camProjMatRight.ConvertToImageProjectionMatrix(in_calibMat),
                in_imageCoordLeft[inout_inlierIndices[it]], in_imageCoordRight[inout_inlierIndices[it]], triangPoint);
            cv::Point3d projPointLeft = camProjectionMatCandidates[candidateId].Apply(triangPoint);
            cv::Point3d projPointRight = camProjMatRight.Apply(triangPoint);

            if (projPointLeft.z > 0.0 && projPointRight.z > 0)
            {
                numVotes[candidateId] += 1;
            }
        }
    }

    int64_t bestId = std::distance(numVotes, std::max_element(numVotes, numVotes + 4U));

    std::vector<uint32_t> refinedInliers;
    
    for (uint32_t idx = 0U; idx < inout_inlierIndices.size(); idx++)
    {
        cv::Point3d triangPoint;
        PointTriangulationLinear(camProjectionMatCandidates[bestId].ConvertToImageProjectionMatrix(in_calibMat), camProjMatRight.ConvertToImageProjectionMatrix(in_calibMat),
            in_imageCoordLeft[inout_inlierIndices[idx]], in_imageCoordRight[inout_inlierIndices[idx]], triangPoint);
        
        if (triangPoint.z > 0. && cv::norm(triangPoint) < 1000.)
        {
            out_triangulatedPoints.push_back(triangPoint);
            refinedInliers.push_back(inout_inlierIndices[idx]);
        }
    }

    out_rotMatrix = camProjectionMatCandidates[bestId].GetRotationMat();
    out_translation = camProjectionMatCandidates[bestId].GetTranslation();
    inout_inlierIndices = refinedInliers;

    return inout_inlierIndices.size() > 0U;
}

bool PointTriangulationLinear(const ImageProjectionMatrix& in_projMatLeft, const ImageProjectionMatrix& in_projMatRight, const cv::Point2d& in_imageCoordLeft,
    const cv::Point2d& in_imageCoordRight, cv::Point3d& out_triangulatedPoint)
{
    cv::Mat1d A = cv::Mat1d::zeros(4, 4);

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
    cv::Mat1d U, D, V_t;
    cv::SVDecomp(A, D, U, V_t, cv::SVD::FULL_UV);

    // Solution is given by nullspace of A
    out_triangulatedPoint.x = V_t.row(3)(0, 0) / V_t.row(3)(0, 3);
    out_triangulatedPoint.y = V_t.row(3)(0, 1) / V_t.row(3)(0, 3);
    out_triangulatedPoint.z = V_t.row(3)(0, 2) / V_t.row(3)(0, 3);

    return true;
}


} //namespace Utils
} //namespace VOCPP
