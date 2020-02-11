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
    // TODO: This model does not do any image point normalization and show bad behavior
    //m_epiPolModels.push_back(new PureTranslationModel());
    m_epiPolModels.push_back(new FullFundamentalMat8pt());
    m_epiPolModels.push_back(new NoMotionModel());
}

bool DeltaPoseReconstructor::FeedNextFrame(const Utils::Frame& in_frame, const cv::Mat& in_calibMat)
{
    bool ret = true;

    if (m_detector == NULL || m_descriptor == NULL || m_matcher == NULL)
    {
        std::cout << "[DeltaPoseReconstructor]: Feature handling members could not be instantiated" << std::endl;
        ret = false;
    }
    else if (!in_frame.isValid())
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
        //if (ret && (!m_lastFrame.isValid() || !m_lastFrame.HasValidPose()))
        if (ret && !m_lastFrame.isValid())
        {
            // Calculate transformation from world coordinate system to camera coordinate system
            // For the first frame the origins are aligned, but the camera coordinate system is rotated
            //cv::Mat worldToCamera = Utils::GetFrameRotationZ(Utils::PI) * Utils::GetFrameRotationX(Utils::PI / 2.0F);
            Utils::CameraPose pose(cv::Mat::eye(3, 3, CV_32F), cv::Mat::zeros(3, 1, CV_32F), in_frame.GetId());
        }
        else if (ret && m_lastFrame.isValid())
        {
            // Get matches and draw them
            std::vector<cv::DMatch> matches;
            ret = m_matcher->MatchDesriptions(descriptions, m_descriptionsLastFrame, m_lastFrame.GetId(), matches);
            std::vector<cv::Point2f> pLastFrame;
            std::vector<cv::Point2f> pCurrFrame;
            Utils::ComputeMatchingPoints(validKeypoints, m_keypointsLastFrame, matches, m_lastFrame.GetId(), pCurrFrame, pLastFrame);

            // Calculate rotation and translation from the matches of the two frames
            std::vector<cv::Point2f> pLastFrameInliers;
            std::vector<cv::Point2f> pCurrFrameInliers;
            cv::Mat rotation;
            cv::Mat translation;
            m_optimizer->Run(m_epiPolModels, pCurrFrame, pLastFrame, in_calibMat, pCurrFrameInliers, pLastFrameInliers, translation, rotation);
            /*
            // The rotations have to be concatenated
            cv::Mat concatRot = rotation * m_lastFrame.GetCameraPose().GetOrientation();
            // The translation is given from last frame to current frame in the last frame coordinate system
            // --> Transform the translation vector back to world coordinate system using the inverse rotation of the last frame
            cv::Mat concatCamCenter =  m_lastFrame.GetCameraPose().GetOrientation().t() * translation + m_lastFrame.GetCameraPose().GetCamCenter();
            in_frame.SetCameraPose(Utils::CameraPose(concatRot, concatCamCenter, in_frame.GetId()));
            std::cout << in_frame.GetCameraPose().GetCamCenter().at<float>(0,0) <<" "<< in_frame.GetCameraPose().GetCamCenter().at<float>(1, 0) <<
                " "<< in_frame.GetCameraPose().GetCamCenter().at<float>(2, 0) << std::endl;
            */

            tick.stop();
            std::cout << "[DeltaPoseReconstruction]: Frame processing time: " << tick.getTimeMilli() << std::endl;
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
        m_lastFrame = std::move(in_frame);
        m_descriptionsLastFrame = std::move(descriptions);
        m_keypointsLastFrame = std::move(validKeypoints);
    }

    return ret;
}

} //namespace DeltaPoseReconstruction
} //namespace VOCPP