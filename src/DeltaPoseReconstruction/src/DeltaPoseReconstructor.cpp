/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_DeltaPoseReconstruction/DeltaPoseReconstructor.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include <FullFundamentalMat8pt.h>
#include<EpipolarModel.h>
#include<RansacOptimizer.h>

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

    // Instantiate optimizer
    m_optimizer = new RansacOptimizer();

    m_epiPolModels.clear();
    m_epiPolModels.push_back(new FullFundamentalMat8pt());

    // Instantiate last processed frame as invalid
    m_lastFrame = Utils::Frame();
}

bool DeltaPoseReconstructor::FeedNextFrame(Utils::Frame& in_frame)
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
        ret = m_detector->ExtractKeypoints(in_frame);
        ret = ret && m_descriptor->ComputeDescriptions(in_frame);

        if (ret && m_lastFrame.isValid())
        {
            // Get matches and draw them
            ret = m_matcher->matchDesriptions(in_frame, m_lastFrame);
            
            std::vector<cv::Point2f> pLastFrame;
            std::vector<cv::Point2f> pCurrFrame;
            Utils::ComputeMatchingPoints(in_frame.GetKeypoints(), m_lastFrame.GetKeypoints(), in_frame.GetMatches(), m_lastFrame.GetId(), pCurrFrame, pLastFrame);

            m_optimizer->Run(m_epiPolModels, pCurrFrame, pLastFrame);
            
            cv::Mat matchImage = in_frame.GetImageCopy();
            cv::cvtColor(matchImage, matchImage, cv::COLOR_GRAY2BGR);

            for (auto matches : in_frame.GetMatches())
            {
                // Draw keypoints from current frame
                cv::circle(matchImage, in_frame.GetKeypoints()[matches.queryIdx].pt, 5, cv::Scalar(0, 0.0, 255.0), 2);
                // Draw keypoints from last frame
                cv::circle(matchImage, m_lastFrame.GetKeypoints()[matches.trainIdx].pt, 5, cv::Scalar(0.0, 255.0, 0.0), 2);
                // Draw connecting line
                cv::line(matchImage, in_frame.GetKeypoints()[matches.queryIdx].pt, m_lastFrame.GetKeypoints()[matches.trainIdx].pt, cv::Scalar(0.0, 0.0, 255.0), 2);
            }

            cv::imshow("Optical Flow", matchImage);
            cv::waitKey(1);
        }

        m_lastFrame = in_frame;
    }

    return ret;
}

} //namespace DeltaPoseReconstruction
} //namespace VOCPP