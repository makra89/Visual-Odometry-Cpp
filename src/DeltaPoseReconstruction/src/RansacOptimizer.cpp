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
        const cv::Mat& in_calibMat, std::vector<cv::Point2f>& out_inliersFirst, std::vector<cv::Point2f>& out_inliersSecond, cv::Mat &out_translation, cv::Mat& out_rotation)
{
    // Save best model according to Plunder Score
    int bestModelId = -1;
    std::vector<int> inliers;

    // Set from outside, or set here hardcoded?
    // This is the gaussian noise in the distance to the epipolar line
    // The models have to scale this value by the number of codimensions of their variety
    const float assumedDistanceError = 1.0;

    int bestPlunderScore = 10000;
    int testedModels = 0;
    for (auto model : in_testedModels)
    {
        
        // The "no motion" model needs a special treatment
        if (model->GetModelType() == EpipolarModel::Types::NoMotionModel)
        {
            std::vector<int> indicesInliers;
            cv::Mat dummySolution = cv::Mat::zeros(3, 3, CV_32F);
            model->Test(in_correspondFirst, in_correspondSecond, dummySolution, assumedDistanceError, indicesInliers);
            int plunderScore = model->ComputePlunderScore(static_cast<int>(indicesInliers.size()), static_cast<int>(in_correspondFirst.size() - indicesInliers.size()));
            if (plunderScore < bestPlunderScore)
            {
                bestPlunderScore = plunderScore;
                inliers = indicesInliers;
                bestModelId = testedModels;
            }
        }

        // Calculate number of necessary iterations
        // We want 95% certainty to have a pure set
        int N = CalculateNecessaryIterations(0.95F, m_outlierRatio, model->GetNumCorrespondences());
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
                int plunderScore = model->ComputePlunderScore(static_cast<int>(indicesInliers.size()), static_cast<int>(in_correspondFirst.size() - indicesInliers.size()));
                if (plunderScore < bestPlunderScore)
                {
                    bestPlunderScore = plunderScore;
                    inliers = indicesInliers;
                    bestModelId = testedModels;
                }

            }
        
        }
        testedModels++;
    }
    // Get all inliers and compute best solution given best model
    out_inliersFirst.reserve(inliers.size());
    out_inliersSecond.reserve(inliers.size());
    for (auto it : inliers)
    {
        out_inliersFirst.push_back(in_correspondFirst[it]);
        out_inliersSecond.push_back(in_correspondSecond[it]);
    }

    // TODO: Here we should do something like a "refinement" step and calculate a better estimate
    // using a more sophisticated method starting from the best solution
    std::vector<cv::Mat> bestSolution;
    in_testedModels[bestModelId]->Compute(out_inliersFirst, out_inliersSecond, bestSolution);

    // TODO: At some point we have to deal with more than one solution --> for now we only have one
    bool ret = in_testedModels[bestModelId]->DecomposeSolution(bestSolution[0], in_calibMat, out_inliersFirst, out_inliersSecond, out_translation, out_rotation);
    
    // We want to output the translation vector from second to first camera center in the second camera coordinate system
    out_translation = -(out_rotation.t() * out_translation);

    // Update outlier ratio estimate
    UpdateOutlierRatio(1.0F - static_cast<float>(inliers.size()) / static_cast<float>(in_correspondFirst.size()));
    return ret;
}

int  RansacOptimizer::CalculateNecessaryIterations(const float in_prob, const float in_outlierRatio, const int in_numCorrespondences)
{
    return static_cast<int>(std::log(1 - in_prob) / std::log(1.0 - std::pow(1.0 - in_outlierRatio, in_numCorrespondences)));
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP