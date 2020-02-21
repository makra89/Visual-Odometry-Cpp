/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_DeltaPoseReconstruction/DeltaPoseReconstructor.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <FullFundamentalMat8pt.h>
#include <PureTranslationModel.h>
#include <NoMotionModel.h>
#include<EpipolarModel.h>
#include<RansacOptimizer.h>
#include <LocalMap.h>
#include <Vocpp_Utils/FrameRotations.h>
#include <Vocpp_Utils/NumericalUtilities.h>
#include <cmath>
#include <opencv2/opencv.hpp>

#include <iostream>


namespace VOCPP
{
namespace DeltaPoseReconstruction
{

DeltaPoseReconstructor::DeltaPoseReconstructor()
{
    // Instantiate all feature handling members
    m_detector = FeatureHandling::InstantiateFeatureDetector("Harris");
    m_descriptor = FeatureHandling::InstantiateFeatureDescriptor("Brief");
    m_matcher = FeatureHandling::InstantiateFeatureMatcher("BruteForce");

    // Instantiate optimizer with an initial estimate of 30% outlier ratio
    m_optimizer = new RansacOptimizer(0.3F, 0.01F);
    // Instantiate local map
    m_localMap = new LocalMap();

    m_epiPolModels.clear();
    m_epiPolModels.push_back(new PureTranslationModel());
    m_epiPolModels.push_back(new FullFundamentalMat8pt());
    m_epiPolModels.push_back(new NoMotionModel());

    m_lastFrameId = s_invalidFrameId;
    m_lastDeltaPose = DeltaCameraPose();
    m_lastPose = CameraPose();
    m_lastOrientationWcs = cv::Mat1f::eye(3, 3);
    m_lastPosWcs = cv::Mat1f::zeros(3, 1);
}


DeltaPoseReconstructor::~DeltaPoseReconstructor()
{
    for (auto model : m_epiPolModels)
    {
        delete model;
    }
    m_epiPolModels.clear();

    delete m_localMap;
    delete m_optimizer;

    delete m_matcher;
    delete m_descriptor;
    delete m_detector;
}

bool DeltaPoseReconstructor::FeedNextFrame(const Frame& in_frame, const cv::Mat1f& in_calibMat)
{
    bool ret = true;

    if (m_detector == NULL || m_descriptor == NULL || m_matcher == NULL)
    {
        std::cout << "[DeltaPoseReconstructor]: Feature handling members could not be instantiated" << std::endl;
        ret = false;
    }
    else if (!in_frame.IsValid())
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
        std::vector<cv::KeyPoint> keypoints;
        ret = m_detector->ExtractKeypoints(in_frame, keypoints);
        std::vector<cv::KeyPoint> validKeypoints;
        std::vector<cv::Mat> descriptions;
        ret = ret && m_descriptor->ComputeDescriptions(in_frame, keypoints, descriptions, validKeypoints);
        
        // If the this is the first frame, or the last frame did not have a valid pose,
        // then set the pose the the center of the world coordinate system
        // TODO: Set it to the last valid pose
        //if (ret && (!m_lastFrame.IsValid() || !m_lastFrame.HasValidPose()))
        if (ret && !IsValidFrameId(m_lastFrameId))
        {
            // Return valid delta pose for first processed frame, but indicate no movement
            m_lastDeltaPose = DeltaCameraPose();
            m_lastOrientationWcs = cv::Mat1f::eye(3, 3);
            m_lastPosWcs = cv::Mat1f::zeros(3, 1);
        }
        else if (ret && IsValidFrameId(m_lastFrameId))
        {
            // Get matches and draw them
            std::vector<cv::DMatch> matches;
            ret = m_matcher->MatchDesriptions(descriptions, m_descriptionsLastFrame, m_lastFrameId, matches);
            std::vector<cv::Point2f> pLastFrame;
            std::vector<cv::Point2f> pCurrFrame;
            Utils::ComputeMatchingPoints(validKeypoints, m_keypointsLastFrame, matches, m_lastFrameId, pCurrFrame, pLastFrame);

            // Calculate rotation and translation from the matches of the two frames
            std::vector<cv::Point2f> pLastFrameInliers;
            std::vector<cv::Point2f> pCurrFrameInliers;
            cv::Mat1f rotation;
            cv::Mat1f translation;
            m_optimizer->Run(m_epiPolModels, pCurrFrame, pLastFrame, in_calibMat, pCurrFrameInliers, pLastFrameInliers, translation, rotation);

            // Up to now the rotation and translation are in the camera coordinate system
            // We want to transform it into the body system in which the x-Axis is aligned
            // with the longitudinal body axis
            cv::Mat1f rotBodyToCamera = Utils::GetFrameRotationZ(-Utils::PI / 2.0F) * Utils::GetFrameRotationY(Utils::PI / 2.0F);
            cv::Mat1f rotationBodySyst = rotBodyToCamera.t() * (rotation * rotBodyToCamera);
            translation = rotBodyToCamera.t() * translation;

            cv::Vec3f eulerAngles = Utils::ExtractRollPitchYaw(rotationBodySyst);
            DeltaOrientation deltaOrientation = { eulerAngles[0], eulerAngles[1] , eulerAngles[2] };
            DeltaTranslation deltaTranslation = { translation(0,0), translation(1,0) , translation(2,0) };
            m_lastDeltaPose = DeltaCameraPose(deltaTranslation, deltaOrientation);

            // Concatenate delta poses to get absolute pose in world coordinate system
            m_lastOrientationWcs = rotationBodySyst * m_lastOrientationWcs;
            cv::Vec3f eulerAnglesWcs = Utils::ExtractRollPitchYaw(m_lastOrientationWcs);
            // The translation is given from last frame to current frame in the last frame coordinate system
            // --> Transform the translation vector back to world coordinate system using the inverse rotation of the last frame
            m_lastPosWcs = m_lastOrientationWcs.t() * translation + m_lastPosWcs;

            Orientation currentOrientWcs = { eulerAnglesWcs[0], eulerAnglesWcs[1] , eulerAnglesWcs[2] };
            Translation currentPosWcs = { m_lastPosWcs(0,0), m_lastPosWcs(1,0) , m_lastPosWcs(2,0) };
            m_lastPose = CameraPose(currentPosWcs, currentOrientWcs);

            tick.stop();
            //std::cout << "[DeltaPoseReconstruction]: Frame processing time: " << tick.getTimeMilli() << std::endl;
            cv::Mat matchImage = in_frame.GetImageCopy();
            cv::cvtColor(matchImage, matchImage, cv::COLOR_GRAY2BGR);

            for (int it = 0; it < pLastFrameInliers.size(); it++)
            {

                // Draw keypoints from current frame
                cv::circle(matchImage, pCurrFrameInliers[it], 5, cv::Scalar(0, 0.0, 255.0), 2);
                // Draw keypoints from last frame
                cv::circle(matchImage, pLastFrameInliers[it], 5, cv::Scalar(0.0, 255.0, 0.0), 2);
                // Draw connecting line
                cv::line(matchImage, pCurrFrameInliers[it], pLastFrameInliers[it], cv::Scalar(0.0, 0.0, 255.0), 2);
            }

         
            cv::imshow("Optical Flow", matchImage);
            cv::waitKey(1);
        }

        // Save last frame, descriptions and keypoints
        m_lastFrameId = in_frame.GetId();
        m_descriptionsLastFrame = std::move(descriptions);
        m_keypointsLastFrame = std::move(validKeypoints);
    }

    return ret;
}

} //namespace DeltaPoseReconstruction
} //namespace VOCPP