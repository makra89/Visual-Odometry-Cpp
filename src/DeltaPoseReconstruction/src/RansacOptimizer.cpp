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

int RansacOptimizer::Run(const std::vector<EpipolarModel*>& in_testedModels, const std::vector<cv::Point2f>& in_correspondFirst, const std::vector<cv::Point2f>& in_correspondSecond)
{
    bool ret = true;
    
    // Save best model according to Plunder Score (to be implemented)
    EpipolarModel::Types bestModelType = EpipolarModel::Types::None;
    std::vector<int> inliers;

    for (auto model : in_testedModels)
    {
        // Number of iterations for model
        // Do not hardcode, but calculate it!
        int N = 10;
        
        for (int modelIt = 0; modelIt < N; modelIt++)
        {
            // Randomly draw a set of correspondences
            std::vector<cv::Point2f> corrFirstDrawn;
            std::vector<cv::Point2f> corrSecondDrawn;
            for (int it = 0; it < static_cast<int>(model->GetNumCorrespondences()); it++)
            {
                int index = Utils::DrawIntInRange(0, corrFirstDrawn.size());
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
                model->Test(corrFirstDrawn, corrSecondDrawn, solution, indicesInliers);

                // Find best solution for this particular model according to inliers
            }
        
        }

    }

    // Use best model in order to refine this particular model using ALL inlier correspondences

    return ret;
}


} //namespace DeltaPoseReconstruction
} //namespace VOCPP