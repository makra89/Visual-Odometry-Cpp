/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <RansacOptimizer.h>
#include <Vocpp_Utils/NumericalUtilities.h>

#include <iostream>

namespace VOCPP
{
namespace DeltaPoseReconstruction
{

int RansacOptimizer::Run(const std::vector<EpipolarModel*>& in_testedModels, const std::vector<cv::Point2f>& in_correspondFirst, const std::vector<cv::Point2f>& in_correspondSecond,
    std::vector<cv::Point2f>& out_inliersFirst, std::vector<cv::Point2f>& out_inliersSecond, cv::Mat& out_bestSolution)
{
    bool ret = true;
    
    // Save best model according to Plunder Score (to be implemented)
    EpipolarModel::Types bestModelType = EpipolarModel::Types::None;
    std::vector<int> inliers;

    // Set from outside, or set here hardcoded?
    // This is the gaussian noise in the distance to the epipolar line
    // The models have to scale this value by the number of codimensions of their variety
    const float assumedDistanceError = 1.0;

    for (auto model : in_testedModels)
    {
        int bestNumInliers = 0;

        // Calculate number of necessary iterations
        // We want 95% certainty to have a pure set
        int N = CalculateNecessaryIterations(static_cast<float>(0.95), m_outlierRatio, model->GetNumCorrespondences());
        for (int modelIt = 0; modelIt < N; modelIt++)
        {
                        
            // Randomly draw a set of correspondences
            std::vector<cv::Point2f> corrFirstDrawn;
            std::vector<cv::Point2f> corrSecondDrawn;
            corrFirstDrawn.reserve(model->GetNumCorrespondences());
            corrSecondDrawn.reserve(model->GetNumCorrespondences());

            for (int it = 0; it < static_cast<int>(model->GetNumCorrespondences()); it++)
            {
                int index = Utils::DrawIntInRange(0, static_cast<int>(in_correspondFirst.size()));
                corrFirstDrawn.push_back(in_correspondFirst[index]);
                corrSecondDrawn.push_back(in_correspondSecond[index]);
            }

            // Compute solution(s) for model
            std::vector<cv::Mat> solutions;
            model->Compute(corrFirstDrawn, corrSecondDrawn, solutions);

            // Test with full set for all solutions and get Inlier
            for (auto solution : solutions)
            {
                std::vector<int> indicesInliers;
                model->Test(in_correspondFirst, in_correspondSecond, solution, assumedDistanceError, indicesInliers);
                
                if (indicesInliers.size() > bestNumInliers)
                {
                    bestNumInliers = static_cast<int>(indicesInliers.size());
                    inliers = indicesInliers;
                }
            }
        
        }
    }

    // Get all inliers and compute best solution given best model
    out_inliersFirst.reserve(inliers.size());
    out_inliersSecond.reserve(inliers.size());
    for (auto it : inliers)
    {
        out_inliersFirst.push_back(in_correspondFirst[it]);
        out_inliersSecond.push_back(in_correspondSecond[it]);
    }

    std::vector<cv::Mat> bestSolution;
    in_testedModels[0]->Compute(out_inliersFirst, out_inliersSecond, bestSolution);
    out_bestSolution = bestSolution[0];

    // Update outlier ratio estimate
    UpdateOutlierRatio(static_cast<float>(1.0) - static_cast<float>(inliers.size()) / static_cast<float>(in_correspondFirst.size()));

    return ret;
}

int  RansacOptimizer::CalculateNecessaryIterations(const float in_prob, const float in_outlierRatio, const int in_numCorrespondences)
{
    return static_cast<int>(std::log(1 - in_prob) / std::log(1.0 - std::pow(1.0 - in_outlierRatio, in_numCorrespondences)));
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP