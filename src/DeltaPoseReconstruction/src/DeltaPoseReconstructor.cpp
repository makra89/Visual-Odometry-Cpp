/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_DeltaPoseReconstruction/DeltaPoseReconstructor.h>
#include <EpipolarSolution.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <Vocpp_Utils/ConversionUtils.h>
#include <Vocpp_Utils/FrameRotations.h>
#include <Vocpp_Utils/NumericalUtilities.h>

#include <cmath>
#include <opencv2/opencv.hpp>

#include <iostream>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

DeltaPoseReconstructor::DeltaPoseReconstructor() :
    m_detectorDescriptor(),
    m_matcher(),
    // Minimum number of tracked landmarks = 10
    m_localMap(10U),
    m_descriptionsLastFrame(),
    m_lastFrameId(s_invalidFrameId),
    m_lastDeltaPose(),
    m_lastPose(),
    m_lastOrientationWcs(),
    m_lastPosWcs(),
    m_lastProjectionMat()
{
    Reset();
}

void DeltaPoseReconstructor::Reset()
{
    m_lastFrameId = s_invalidFrameId;
    // Reset pose back to WCS coordinate center
    m_lastDeltaPose = DeltaCameraPose();
    m_lastPose = CameraPose();
    m_lastOrientationWcs = cv::Mat1f::eye(3, 3);
    m_lastPosWcs = cv::Mat1f::zeros(3, 1);

    // Clear last descriptions and reset local map
    m_descriptionsLastFrame.clear();
    m_localMap.Reset();
}

DeltaPoseReconstructor::~DeltaPoseReconstructor()
{
    Reset();
}

bool DeltaPoseReconstructor::FeedNextFrame(const Frame& in_frame, const cv::Mat1f& in_calibMat, bool in_debugOutputFlag)
{
    bool ret = true;

    if (!in_frame.IsValid())
    {
        std::cout << "[DeltaPoseReconstructor]: Invalid frame provided" << std::endl;
        ret = false;
    }
    else
    {
        // Measure frame processing time
        cv::TickMeter tick;
        tick.start();

        // Get features and descriptions
        std::vector<FeatureHandling::BinaryFeatureDescription> descriptions;
        ret = ret && m_detectorDescriptor.ExtractFeatureDescriptions(in_frame, 750U, descriptions);
        // If the this is the first frame, or the last frame did not have a valid pose,
        // then set the pose to the center of the world coordinate system
        // TODO: Set it to the last valid pose
        if (ret)
        {
            if (!IsValidFrameId(m_lastFrameId))
            {
                // Return valid delta pose for first processed frame, but indicate no movement
                m_lastDeltaPose = DeltaCameraPose();
                m_lastOrientationWcs = cv::Mat1f::eye(3, 3);
                m_lastPosWcs = cv::Mat1f::zeros(3, 1);
                m_lastProjectionMat = Utils::ImageProjectionMatrix(m_lastOrientationWcs, m_lastPosWcs, in_calibMat);
            }
            else
            {
                // Get matches and draw them
                std::vector<FeatureHandling::BinaryDescriptionMatch> matches;
                ret = m_matcher.MatchDesriptions(descriptions, m_descriptionsLastFrame, matches);
                std::vector<cv::Point2f> pLastFrame;
                std::vector<cv::Point2f> pCurrFrame;
                FeatureHandling::GetMatchingPoints(matches, pCurrFrame, pLastFrame);

                // Calculate rotation and translation from the matches of the two frames
                std::vector<unsigned int> inlierMatchIndices;
                cv::Mat1f rotation;
                cv::Mat1f translation;
                std::vector<cv::Point3f> triangulatedPoints;
                RecoverPoseRansac(pCurrFrame, pLastFrame, in_calibMat, inlierMatchIndices, translation, rotation, triangulatedPoints);

                ////////////////////////////////////////////////////////////////////////////////////////////////////
                ///////                             RAW TRANSLATION (Without applying a relative scale        //////
                ////////////////////////////////////////////////////////////////////////////////////////////////////
                
                // Up to now the rotation and translation are in the camera coordinate system
                // We want to transform it into the body system in which the x-Axis is aligned
                // with the longitudinal body axis
                cv::Mat1f rotBodyToCamera = Utils::GetFrameRotationZ(-Utils::PI / 2.0F) * Utils::GetFrameRotationY(Utils::PI / 2.0F);
                cv::Mat1f rotationBodySyst = rotBodyToCamera.t() * (rotation * rotBodyToCamera);
                translation = rotBodyToCamera.t() * translation;

                // Concatenate delta poses to get absolute pose in world coordinate system
                m_lastOrientationWcs = rotationBodySyst * m_lastOrientationWcs;
                // The translation is given from last frame to current frame in the last frame coordinate system
                // --> Transform the translation vector back to world coordinate system using the inverse rotation of the last frame
                cv::Mat1f rawPosWcs = m_lastOrientationWcs.t() * translation + m_lastPosWcs;
                
                ////////////////////////////////////////////////////////////////////////////////////////////////////
                ///////                            TRANSLATION REFINEMENT (with relative scale                //////
                ////////////////////////////////////////////////////////////////////////////////////////////////////
                
                // Create a vector with all inlier matches
                std::vector<FeatureHandling::BinaryDescriptionMatch> inlierMatches;
                for (auto matchIdx : inlierMatchIndices)
                {
                    inlierMatches.push_back(matches[matchIdx]);
                }

                std::vector<LandmarkPosition> landmarks;
                for (auto point : triangulatedPoints)
                {
                    landmarks.push_back(LandmarkPosition{ point.x, point.y, point.z });
                }

                // Add triangulated landmarks to local map
                m_localMap.InsertLandmarks(landmarks, inlierMatches, in_frame.GetId());
                // And calculate scale, apply it to translation vector
                float scale = 0.0;
                if (m_localMap.GetLastRelativeScale(m_lastFrameId, in_frame.GetId(), scale))
                {
                    translation = translation * scale;
                }
                

                ////////////////////////////////////////////////////////////////////////////////////////////////////
                ///////                            REFINED POSE CALCULATION                                   //////
                ////////////////////////////////////////////////////////////////////////////////////////////////////

                // Calculate last delta pose
                cv::Vec3f eulerAngles = Utils::ExtractRollPitchYaw(rotationBodySyst);
                DeltaOrientation deltaOrientation = { eulerAngles[0], eulerAngles[1] , eulerAngles[2] };
                DeltaTranslation deltaTranslation = { translation(0,0), translation(1,0) , translation(2,0) };
                m_lastDeltaPose = DeltaCameraPose(deltaTranslation, deltaOrientation);

                // And last absolute pose
                cv::Vec3f eulerAnglesWcs = Utils::ExtractRollPitchYaw(m_lastOrientationWcs);
                m_lastPosWcs = m_lastOrientationWcs.t() * translation + m_lastPosWcs;
                Orientation currentOrientWcs = { eulerAnglesWcs[0], eulerAnglesWcs[1] , eulerAnglesWcs[2] };
                Translation currentPosWcs = { m_lastPosWcs(0,0), m_lastPosWcs(1,0) , m_lastPosWcs(2,0) };
                m_lastPose = CameraPose(currentPosWcs, currentOrientWcs);

                tick.stop();
                // Only show debug output if requested
                if (in_debugOutputFlag)
                {
                    std::cout << "[DeltaPoseReconstruction]: Frame processing time: " << tick.getTimeMilli() << std::endl;
                    cv::Mat matchImage = in_frame.GetImageCopy();
                    cv::cvtColor(matchImage, matchImage, cv::COLOR_GRAY2BGR);
                    for (auto matchIdx : inlierMatchIndices)
                    {
                        // Draw keypoints from current frame
                        cv::circle(matchImage, pCurrFrame[matchIdx], 5, cv::Scalar(0, 0.0, 255.0), 2);
                        // Draw keypoints from last frame
                        cv::circle(matchImage, pLastFrame[matchIdx], 5, cv::Scalar(0.0, 255.0, 0.0), 2);
                        // Draw connecting line
                        cv::line(matchImage, pCurrFrame[matchIdx], pLastFrame[matchIdx], cv::Scalar(0.0, 0.0, 255.0), 2);
                    }

                    cv::imshow("Optical Flow", matchImage);
                    cv::waitKey(1);
                }
            }

            // Save last frame, descriptions and keypoints
            m_lastFrameId = in_frame.GetId();
            m_descriptionsLastFrame = std::move(descriptions);
        }
        else
        {
            // Reset all members
            Reset();
        }
    }

    return ret;
}

} //namespace DeltaPoseReconstruction
} //namespace VOCPP