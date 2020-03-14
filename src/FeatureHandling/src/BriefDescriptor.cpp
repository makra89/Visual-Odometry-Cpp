/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/BriefDescriptor.h>
#include<Vocpp_Utils/NumericalUtilities.h>
#include<Vocpp_Utils/IntImage.h>

#include <iostream>
#include<random>
#include<fstream>

namespace VOCPP
{
namespace FeatureHandling
{


BriefDescriptor::BriefDescriptor(const int& in_randomPairDrawRadius, const int& in_areaDetRadius,
    const bool& in_trainingMode, std::string in_filePath, const int& in_numFramesForTraining) :
    m_randomPairDrawRadius(in_randomPairDrawRadius),
    m_areaDetRadius(in_areaDetRadius),
    m_pairs(),
    m_trainingMode(in_trainingMode),
    m_numFramesForTraining(in_numFramesForTraining),
    m_trainPositions()
{
    if (m_trainingMode)
    {
        m_patternOutFile.open(in_filePath, std::ios::out | std::ios::trunc);
        DrawTrainPairs();
    }
    else
    {
        GetTestPositions();
    }
}

bool BriefDescriptor::ComputeDescriptions(const Frame& in_frame, const std::vector<Feature>& in_features,
    std::vector<BinaryFeatureDescription>& out_descriptions)
{
    bool ret = false;

    if (in_features.size() == 0)
    {    
        std::cout << "[BriefDescriptor]: No keypoints found in provided frame" << std::endl;
        return false;
    }

    if (out_descriptions.size() != 0)
    {
        std::cout << "[BriefDescriptor]: Frame with non-empty description vector provided, will be erased" << std::endl;
        return false;
    }

    Utils::IntImage integImage(in_frame.GetImage());

    // Loop over keypoints
    for (auto feature : in_features)
    {
        std::vector<bool> description;
        description.reserve(s_numRandomPairs);
        
        if (m_trainingMode && ( (feature.imageCoordX < 3 * m_randomPairDrawRadius || (feature.imageCoordX + 3 * m_randomPairDrawRadius) > in_frame.GetImage().cols)
            || (feature.imageCoordY < 3 * m_randomPairDrawRadius || (feature.imageCoordY + 3 * m_randomPairDrawRadius) > in_frame.GetImage().rows)))
        {
            continue;
        }
        else if (m_trainingMode && in_frame.GetId() > m_numFramesForTraining)
        {
            std::cout << "[BriefDescriptor - TrainingMode]: Finished detection, now finding best pattern " << std::endl;
            FindBestPattern(m_trainPositions, m_patternOutFile);
            m_patternOutFile.close();
            std::cout << "[BriefDescriptor - TrainingMode]: Finished training " << std::endl;
            break;
        }
        else
        {
            // serve goddamn MISRA
        }


        // Loop over pairs
        int pairId = 0;
        int numSucessfulPairs = 0U;
        for (auto pair : m_pairs)
        {
            PointPair rotPair = RotatePair(feature.angle, pair);
            
            int indFirstX = static_cast<int>(rotPair.x1 + feature.imageCoordX);
            int indFirstY = static_cast<int>(rotPair.y1 + feature.imageCoordY);
            int indSecX = static_cast<int>(rotPair.x2 + feature.imageCoordX);
            int indSecY = static_cast<int>(rotPair.y2 + feature.imageCoordY);

            // Check whether all indices are within range
            const bool inRangeFirstX = (indFirstX >= 0) && (indFirstX < in_frame.GetImage().size[1]);
            const bool inRangeFirstY = (indFirstY >= 0) && (indFirstY < in_frame.GetImage().size[0]);
            const bool inRangeSecX = (indSecX >= 0) && (indSecX < in_frame.GetImage().size[1]);
            const bool inRangeSecY = (indSecY >= 0) && (indSecY < in_frame.GetImage().size[0]);

            if (inRangeFirstX && inRangeFirstY && inRangeSecX && inRangeSecY)
            {
                // Compare intensities and append bin to description
                float areaFirst = 0.0;
                integImage.GetAreaAroundPixel(indFirstY, indFirstX, m_areaDetRadius, areaFirst);
                float areaSecond = 0.0;
                integImage.GetAreaAroundPixel(indSecY, indSecX, m_areaDetRadius, areaSecond);
                
                bool bin = areaFirst > areaSecond ? true : false;
                description.push_back(bin);
                numSucessfulPairs++;

                if (m_trainingMode && m_trainPositions.size() <= pairId)
                {
                    m_trainPositions.push_back(TrainPosition(pairId, pair));
                    m_trainPositions[pairId].AddVal(bin);

                }
                else if(m_trainingMode)
                {
                    m_trainPositions[pairId].AddVal(bin);
                }
                else
                {
                    // again MISRA
                }

            }
            // Now this is really bad, for now just deactivate training mode
            // This should really never happen
            else if(m_trainingMode)
            {
                std::cout << "[BriefDescriptor - TrainingMode]: Found a single test pattern that got rejected, training not possible anymore " << feature.id << std::endl;
                m_trainingMode = false;
                break;
            }
            else
            {
                // Hello MISRA
            }

            pairId++;
        }

        if (numSucessfulPairs == s_numRandomPairs)
        {
            out_descriptions.push_back(BinaryFeatureDescription(feature, description));
            ret = true;
        }
    }

    return ret;
}

void BriefDescriptor::GetTestPositions()
{
    m_pairs.clear();
    
    for (int i = 0; i < 4 * s_numRandomPairs; i += 4)
    {
        m_pairs.push_back(PointPair{ static_cast<float>(s_testPattern[i]),
                                     static_cast<float>(s_testPattern[i + 1]), 
                                     static_cast<float>(s_testPattern[i + 2]),
                                     static_cast<float>(s_testPattern[i + 3])});
    }
}


void BriefDescriptor::DrawTrainPairs()
{
    m_pairs.clear();
    
    const int drawRadius = m_randomPairDrawRadius - m_areaDetRadius;

    // Get set of possible test patch locations
    std::vector<cv::Point2i> possiblePoint;
    for (int i = -drawRadius; i <= drawRadius; i++)
    {
        for (int j = -drawRadius; j <= drawRadius; j++)
        {
            possiblePoint.push_back(cv::Point2i(i, j));
        }
    }

    // And build all possible combinations (without replacement)
    for (int i = 0; i < possiblePoint.size(); i++)
    {
        for (int j = i + 1; j < possiblePoint.size(); j++)
        {
            // Only choose test locations without overlap
            if (std::abs(possiblePoint[i].x - possiblePoint[j].x) <= (2 * m_areaDetRadius)
                && std::abs(possiblePoint[i].y - possiblePoint[j].y) <= (2 * m_areaDetRadius))
            {
            }
            else
            {
                m_pairs.push_back(PointPair{ static_cast<float>(possiblePoint[i].x), static_cast<float>(possiblePoint[i].y),
                            static_cast<float>(possiblePoint[j].x), static_cast<float>(possiblePoint[j].y) });
            }
        }
    }
}

BriefDescriptor::PointPair BriefDescriptor::RotatePair(const float& in_angle, const PointPair& in_pair)
{
    PointPair rotPair;
    rotPair.x1 = std::cos(in_angle) * in_pair.x1 - std::sin(in_angle) * in_pair.y1;
    rotPair.y1 = std::cos(in_angle) * in_pair.y1 + std::sin(in_angle) * in_pair.x1;
    rotPair.x2 = std::cos(in_angle) * in_pair.x2 - std::sin(in_angle) * in_pair.y2;
    rotPair.y2 = std::cos(in_angle) * in_pair.y2 + std::sin(in_angle) * in_pair.x2;

    return rotPair;
}

float BriefDescriptor::CalculatePearsonCorr(const TrainPosition& in_first, const TrainPosition& in_second)
{
    float pearson = -999.0;

    // Score computation is only possible for test vectors of equal length
    // Additionally make sure that the result vector is not empty
    if (in_first.m_testResults->size() == in_second.m_testResults->size()
        && in_first.m_testResults->size() >= 10)
    {
        float nom = 0.0;
        float denomFirst = 0.0;
        float denomSec = 0.0;
        float firstMean = in_first.GetMean();
        float secMean = in_second.GetMean();
        for (int i = 0; i < in_first.m_testResults->size(); i++)
        {
            nom += (((*in_first.m_testResults)[i] - firstMean) * ((*in_second.m_testResults)[i] - secMean));
            denomFirst += std::pow((*in_first.m_testResults)[i] - in_first.GetMean(), 2);
            denomSec += std::pow((*in_second.m_testResults)[i] - in_second.GetMean(), 2);
        }

        pearson = nom / (std::sqrt(denomFirst) * std::sqrt(denomSec));
    }
    else
    {
        std::cout << "[BriefDescriptor - TrainingMode]: Error in pearson correlation calculation, vectors not of same length or too few elements" << std::endl;
    }

    return pearson;
}

void BriefDescriptor::FindBestPattern(const std::vector<TrainPosition>& in_trainPositions, std::ofstream& in_file)
{
    // Sort positions according to distance to mean 0.5
    std::vector<TrainPosition> copy = in_trainPositions;
    std::sort(copy.rbegin(), copy.rend(), std::greater<TrainPosition>());

    // Copy first position to best pattern vector (we have to start somewhere)
    std::vector<TrainPosition> bestPatterns;
    bestPatterns.push_back(copy[0]);

    float thresh = 0.1F;
    while (bestPatterns.size() < BriefDescriptor::s_numRandomPairs)
    {
        for (const auto pos : copy)
        {
            if (bestPatterns.size() >= BriefDescriptor::s_numRandomPairs)
            {
                break;
            }
            
            // Find test position with lowest correlation to existing "best positions"
            float maximumCorr = 0.0;
            for (const auto bestPos : bestPatterns)
            {
                const float corr = CalculatePearsonCorr(bestPos, pos);
                if (std::abs(corr) > maximumCorr)
                {
                    maximumCorr = std::abs(corr);
                }
            }

            // We found one --> add it to "best positions"
            if (maximumCorr < thresh)
            {
                bestPatterns.push_back(pos);
                std::cout << "[BriefDescriptor - TrainingMode]: Found test position, looking for " << BriefDescriptor::s_numRandomPairs  - bestPatterns.size() <<" more." << std::endl;
                in_file << "/* mean: " << pos.GetMean() << " var: " << pos.GetVariance() << " maxCorr: " << maximumCorr<<" */ " <<
                    pos.m_pair.x1 << ", " << pos.m_pair.y1 << ", " << pos.m_pair.x2 << ", " << pos.m_pair.y2 << ", " << std::endl;
            }
        }

        // Increase threshold if we did not yet found enough
        thresh += 0.05F;
    }
}

/* Test positions used for intensity comparison
Trained with PASCAL 2006 image set (like in the original ORB paper */
const int BriefDescriptor::s_testPattern[256 * 4] =
{
    /* mean: 0.5 var: 0.250547 maxCorr: 0.0393013 */ 1, -1, 11, -10,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.0393013 */ -11, 3, -5, -4,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.0742358 */ 8, -5, 13, 0,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.0742358 */ 2, 6, 5, 13,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.0917031 */ 8, 11, 13, 10,
    /* mean: 0.495633 var: 0.250528 maxCorr: 0.0917066 */ -3, -13, -2, -8,
    /* mean: 0.519651 var: 0.25016 maxCorr: 0.0961442 */ -13, -13, -8, -11,
    /* mean: 0.519651 var: 0.25016 maxCorr: 0.0964914 */ 1, -4, 1, 7,
    /* mean: 0.478166 var: 0.250069 maxCorr: 0.0917906 */ 7, -13, 12, -13,
    /* mean: 0.526201 var: 0.249859 maxCorr: 0.0918292 */ -13, 12, -10, 4,
    /* mean: 0.550218 var: 0.24802 maxCorr: 0.0877801 */ 12, 8, 13, 13,
    /* mean: 0.432314 var: 0.245956 maxCorr: 0.0881476 */ 2, 13, 8, 12,
    /* mean: 0.368996 var: 0.233346 maxCorr: 0.0950229 */ -5, -10, 2, -11,
    /* mean: 0.275109 var: 0.19986 maxCorr: 0.0856551 */ 0, -10, 5, -4,
    /* mean: 0.144105 var: 0.123609 maxCorr: 0.0870387 */ -5, 0, 0, 3,
    /* mean: 0.0567686 var: 0.0536633 maxCorr: 0.0943563 */ -3, -3, 4, 0,
    /* mean: 0.508734 var: 0.250471 maxCorr: 0.144127 */ 12, 0, 12, 5,
    /* mean: 0.484716 var: 0.250313 maxCorr: 0.139803 */ -1, -3, -1, 3,
    /* mean: 0.467249 var: 0.249472 maxCorr: 0.140039 */ -13, 10, -8, 13,
    /* mean: 0.467249 var: 0.249472 maxCorr: 0.140617 */ -11, -8, -10, -13,
    /* mean: 0.462882 var: 0.249166 maxCorr: 0.148883 */ 12, -12, 13, -6,
    /* mean: 0.417031 var: 0.243648 maxCorr: 0.148431 */ -3, 13, 2, 10,
    /* mean: 0.137555 var: 0.118893 maxCorr: 0.148933 */ -2, 1, 1, -4,
    /* mean: 0.128821 var: 0.112471 maxCorr: 0.147942 */ 0, -3, 3, 2,
    /* mean: 0.497817 var: 0.250542 maxCorr: 0.196832 */ 2, 2, 7, 6,
    /* mean: 0.469432 var: 0.249611 maxCorr: 0.170625 */ -9, -11, -4, -10,
    /* mean: 0.469432 var: 0.249611 maxCorr: 0.196875 */ 10, 9, 13, 4,
    /* mean: 0.552402 var: 0.247796 maxCorr: 0.197595 */ -9, -7, -8, 0,
    /* mean: 0.443231 var: 0.247317 maxCorr: 0.192979 */ 3, -5, 11, -3,
    /* mean: 0.436681 var: 0.246529 maxCorr: 0.183031 */ -7, 13, -2, 12,
    /* mean: 0.565502 var: 0.246248 maxCorr: 0.180596 */ -6, 9, -4, 4,
    /* mean: 0.353712 var: 0.2291 maxCorr: 0.197553 */ -7, -5, -1, 0,
    /* mean: 0.224891 var: 0.174696 maxCorr: 0.198747 */ -5, 6, 9, 0,
    /* mean: 0.20524 var: 0.163473 maxCorr: 0.196358 */ 0, -6, 3, 0,
    /* mean: 0.502183 var: 0.250542 maxCorr: 0.229798 */ 4, 3, 5, -3,
    /* mean: 0.497817 var: 0.250542 maxCorr: 0.241777 */ 3, -7, 5, -13,
    /* mean: 0.508734 var: 0.250471 maxCorr: 0.241204 */ 7, -6, 13, -8,
    /* mean: 0.530568 var: 0.249611 maxCorr: 0.248343 */ 9, 2, 13, -3,
    /* mean: 0.537118 var: 0.249166 maxCorr: 0.245218 */ -13, -7, -8, -6,
    /* mean: 0.458515 var: 0.248822 maxCorr: 0.236624 */ -4, 7, -3, 12,
    /* mean: 0.545852 var: 0.24844 maxCorr: 0.236806 */ -13, 5, -10, 0,
    /* mean: 0.449782 var: 0.24802 maxCorr: 0.245784 */ -13, -4, -12, -9,
    /* mean: 0.561135 var: 0.246802 maxCorr: 0.239875 */ 6, 3, 11, 4,
    /* mean: 0.419214 var: 0.244006 maxCorr: 0.249607 */ 7, -8, 8, -3,
    /* mean: 0.580786 var: 0.244006 maxCorr: 0.245662 */ -3, 6, -2, 1,
    /* mean: 0.401747 var: 0.240872 maxCorr: 0.249404 */ -12, 10, 0, -10,
    /* mean: 0.388646 var: 0.238121 maxCorr: 0.246532 */ 5, 9, 7, 3,
    /* mean: 0.379913 var: 0.236094 maxCorr: 0.248899 */ -8, -3, -2, -6,
    /* mean: 0.340611 var: 0.225086 maxCorr: 0.239175 */ -11, 1, 13, 12,
    /* mean: 0.283843 var: 0.203721 maxCorr: 0.240846 */ 0, 3, 1, -2,
    /* mean: 0.216157 var: 0.169803 maxCorr: 0.246639 */ -13, -12, 4, -2,
    /* mean: 0.133188 var: 0.115701 maxCorr: 0.232907 */ -1, -2, 1, 3,
    /* mean: 0.0218341 var: 0.0214041 maxCorr: 0.221608 */ -3, 0, 3, 1,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.285645 */ 6, -7, 9, 12,
    /* mean: 0.491266 var: 0.250471 maxCorr: 0.292107 */ -13, 5, -10, 10,
    /* mean: 0.489083 var: 0.250429 maxCorr: 0.289288 */ 0, -3, 0, 2,
    /* mean: 0.517467 var: 0.250241 maxCorr: 0.292755 */ 5, 8, 10, 13,
    /* mean: 0.517467 var: 0.250241 maxCorr: 0.294 */ -4, -9, -2, -4,
    /* mean: 0.456332 var: 0.248636 maxCorr: 0.293509 */ -5, -6, -5, 9,
    /* mean: 0.451965 var: 0.248233 maxCorr: 0.27993 */ 4, 8, 9, 8,
    /* mean: 0.438865 var: 0.246802 maxCorr: 0.298395 */ -10, 10, -4, 8,
    /* mean: 0.425764 var: 0.245024 maxCorr: 0.280367 */ 3, -13, 8, -11,
    /* mean: 0.401747 var: 0.240872 maxCorr: 0.299272 */ 4, -13, 9, 4,
    /* mean: 0.399563 var: 0.240439 maxCorr: 0.298663 */ -10, -13, 4, 12,
    /* mean: 0.382096 var: 0.236615 maxCorr: 0.295434 */ -9, 3, -3, 10,
    /* mean: 0.379913 var: 0.236094 maxCorr: 0.297725 */ -2, 13, 13, -5,
    /* mean: 0.379913 var: 0.236094 maxCorr: 0.295613 */ -6, -6, -3, -11,
    /* mean: 0.364629 var: 0.232182 maxCorr: 0.295158 */ 0, -8, 6, -8,
    /* mean: 0.362445 var: 0.231584 maxCorr: 0.288536 */ -6, 7, -1, 7,
    /* mean: 0.358079 var: 0.230361 maxCorr: 0.292077 */ 1, 10, 4, 5,
    /* mean: 0.303493 var: 0.211849 maxCorr: 0.295206 */ -3, -7, 3, -6,
    /* mean: 0.207424 var: 0.164759 maxCorr: 0.291949 */ -3, 3, 0, -2,
    /* mean: 0.203057 var: 0.162179 maxCorr: 0.291953 */ -3, 3, 3, 6,
    /* mean: 0.170306 var: 0.14161 maxCorr: 0.298535 */ -4, -1, 11, 4,
    /* mean: 0.128821 var: 0.112471 maxCorr: 0.290454 */ -3, -4, 1, 3,
    /* mean: 0.122271 var: 0.107555 maxCorr: 0.293825 */ -8, 2, 3, -2,
    /* mean: 0.0371179 var: 0.0358184 maxCorr: 0.286778 */ -2, 2, 3, -1,
    /* mean: 0.010917 var: 0.0108215 maxCorr: 0.273211 */ -3, 0, 2, 0,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.347064 */ -11, -8, -3, -4,
    /* mean: 0.502183 var: 0.250542 maxCorr: 0.34206 */ -13, -9, -11, 13,
    /* mean: 0.504367 var: 0.250528 maxCorr: 0.34059 */ 4, 4, 12, 0,
    /* mean: 0.504367 var: 0.250528 maxCorr: 0.338313 */ -7, -2, -5, 3,
    /* mean: 0.510917 var: 0.250429 maxCorr: 0.327496 */ -12, -3, -12, 2,
    /* mean: 0.484716 var: 0.250313 maxCorr: 0.332618 */ 5, -9, 10, -13,
    /* mean: 0.482533 var: 0.250241 maxCorr: 0.342447 */ 0, -5, 1, -10,
    /* mean: 0.519651 var: 0.25016 maxCorr: 0.338313 */ 1, 3, 2, -6,
    /* mean: 0.521834 var: 0.250069 maxCorr: 0.33004 */ -9, -13, -6, -8,
    /* mean: 0.526201 var: 0.249859 maxCorr: 0.319216 */ -13, 1, -8, 0,
    /* mean: 0.537118 var: 0.249166 maxCorr: 0.341554 */ 10, -1, 11, -6,
    /* mean: 0.445415 var: 0.247561 maxCorr: 0.347928 */ -8, 8, -8, 13,
    /* mean: 0.438865 var: 0.246802 maxCorr: 0.332493 */ 6, 13, 9, 8,
    /* mean: 0.427948 var: 0.245344 maxCorr: 0.335365 */ 6, -12, 9, -7,
    /* mean: 0.572052 var: 0.245344 maxCorr: 0.317727 */ 9, 3, 13, 8,
    /* mean: 0.578603 var: 0.244354 maxCorr: 0.347584 */ 6, -1, 10, 4,
    /* mean: 0.578603 var: 0.244354 maxCorr: 0.349322 */ -9, 5, -7, 0,
    /* mean: 0.578603 var: 0.244354 maxCorr: 0.344055 */ -7, -9, -7, -4,
    /* mean: 0.41048 var: 0.242516 maxCorr: 0.345341 */ 0, -11, 6, -13,
    /* mean: 0.395196 var: 0.239539 maxCorr: 0.32602 */ -3, 9, 2, 13,
    /* mean: 0.606987 var: 0.239075 maxCorr: 0.330806 */ 6, 0, 7, -5,
    /* mean: 0.39083 var: 0.238603 maxCorr: 0.342656 */ -1, -13, 4, -10,
    /* mean: 0.613537 var: 0.237628 maxCorr: 0.339099 */ -2, -4, -2, 1,
    /* mean: 0.364629 var: 0.232182 maxCorr: 0.334049 */ 0, 8, 6, 9,
    /* mean: 0.362445 var: 0.231584 maxCorr: 0.348036 */ -13, 6, 3, 10,
    /* mean: 0.349345 var: 0.227801 maxCorr: 0.341613 */ -11, 1, 6, -13,
    /* mean: 0.349345 var: 0.227801 maxCorr: 0.3488 */ -6, -11, 13, -8,
    /* mean: 0.347162 var: 0.227137 maxCorr: 0.345785 */ -8, 13, 13, 5,
    /* mean: 0.331878 var: 0.222221 maxCorr: 0.337786 */ -5, 1, -3, -8,
    /* mean: 0.272926 var: 0.198872 maxCorr: 0.348002 */ -7, 4, 0, -5,
    /* mean: 0.266376 var: 0.195847 maxCorr: 0.345807 */ -13, -3, 9, -4,
    /* mean: 0.259825 var: 0.192738 maxCorr: 0.349517 */ 1, 5, 5, -1,
    /* mean: 0.253275 var: 0.18954 maxCorr: 0.348904 */ 0, 0, 6, -2,
    /* mean: 0.244541 var: 0.185144 maxCorr: 0.336242 */ -12, 13, 4, 2,
    /* mean: 0.240175 var: 0.18289 maxCorr: 0.344851 */ -4, -8, 4, 4,
    /* mean: 0.187773 var: 0.152848 maxCorr: 0.339264 */ -5, -3, 1, -4,
    /* mean: 0.0480349 var: 0.0458275 maxCorr: 0.333961 */ -2, 3, 3, 1,
    /* mean: 0.0218341 var: 0.0214042 maxCorr: 0.321565 */ -3, 0, 2, 2,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.394676 */ -13, 5, -7, 5,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.381535 */ -9, 8, -2, 0,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.388646 */ 5, 9, 13, -13,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.380625 */ 12, -3, 13, 2,
    /* mean: 0.502183 var: 0.250542 maxCorr: 0.385689 */ 0, -2, 1, 13,
    /* mean: 0.495633 var: 0.250528 maxCorr: 0.390074 */ -9, 0, -3, 0,
    /* mean: 0.50655 var: 0.250503 maxCorr: 0.391117 */ 0, 8, 0, 13,
    /* mean: 0.49345 var: 0.250503 maxCorr: 0.384313 */ 8, -7, 13, -4,
    /* mean: 0.49345 var: 0.250503 maxCorr: 0.38506 */ -13, -1, -10, -6,
    /* mean: 0.4869 var: 0.250375 maxCorr: 0.36257 */ -12, 2, -12, 8,
    /* mean: 0.4869 var: 0.250375 maxCorr: 0.397108 */ 3, -2, 8, 0,
    /* mean: 0.480349 var: 0.25016 maxCorr: 0.399543 */ 1, -1, 2, 4,
    /* mean: 0.478166 var: 0.250069 maxCorr: 0.388966 */ -12, -9, -6, -11,
    /* mean: 0.475983 var: 0.24997 maxCorr: 0.39962 */ -10, 6, -7, -9,
    /* mean: 0.469432 var: 0.249611 maxCorr: 0.393587 */ -11, 13, -6, 13,
    /* mean: 0.465066 var: 0.249324 maxCorr: 0.399483 */ 3, -5, 13, 7,
    /* mean: 0.541485 var: 0.248822 maxCorr: 0.398068 */ -13, -13, -3, 3,
    /* mean: 0.458515 var: 0.248822 maxCorr: 0.390546 */ -13, 8, -13, 13,
    /* mean: 0.543668 var: 0.248636 maxCorr: 0.396083 */ 4, 4, 10, 10,
    /* mean: 0.548035 var: 0.248233 maxCorr: 0.385273 */ 8, 2, 13, 1,
    /* mean: 0.447598 var: 0.247795 maxCorr: 0.397886 */ 5, 8, 12, 5,
    /* mean: 0.556769 var: 0.247318 maxCorr: 0.390796 */ 7, -2, 12, -2,
    /* mean: 0.441048 var: 0.247064 maxCorr: 0.398055 */ 9, 11, 10, 6,
    /* mean: 0.563319 var: 0.246529 maxCorr: 0.392211 */ -13, -4, -8, -1,
    /* mean: 0.434498 var: 0.246248 maxCorr: 0.389466 */ -8, 2, -7, 7,
    /* mean: 0.434498 var: 0.246248 maxCorr: 0.395121 */ -7, 13, -1, 5,
    /* mean: 0.572052 var: 0.245344 maxCorr: 0.392019 */ 10, -4, 11, -9,
    /* mean: 0.419214 var: 0.244007 maxCorr: 0.387981 */ -6, -13, -1, -10,
    /* mean: 0.585153 var: 0.24328 maxCorr: 0.39735 */ -10, 8, -7, 3,
    /* mean: 0.587336 var: 0.242902 maxCorr: 0.387103 */ 7, 1, 7, 6,
    /* mean: 0.598253 var: 0.240872 maxCorr: 0.359874 */ -13, -8, -10, -3,
    /* mean: 0.399563 var: 0.240439 maxCorr: 0.398176 */ -10, -5, -2, 7,
    /* mean: 0.399563 var: 0.240439 maxCorr: 0.39522 */ -2, -13, 1, 0,
    /* mean: 0.39738 var: 0.239994 maxCorr: 0.398953 */ 1, -12, 13, -1,
    /* mean: 0.606987 var: 0.239075 maxCorr: 0.393391 */ 8, 0, 9, -13,
    /* mean: 0.388646 var: 0.238121 maxCorr: 0.394179 */ -2, 7, 6, -8,
    /* mean: 0.377729 var: 0.235564 maxCorr: 0.393522 */ -6, -7, -1, -6,
    /* mean: 0.377729 var: 0.235564 maxCorr: 0.391809 */ -12, -13, 9, -13,
    /* mean: 0.626638 var: 0.234474 maxCorr: 0.399474 */ 5, 0, 10, -3,
    /* mean: 0.360262 var: 0.230978 maxCorr: 0.361383 */ -4, 9, 1, 6,
    /* mean: 0.358079 var: 0.230361 maxCorr: 0.396522 */ 4, -9, 4, -4,
    /* mean: 0.355895 var: 0.229736 maxCorr: 0.383294 */ 3, -4, 5, 1,
    /* mean: 0.347162 var: 0.227137 maxCorr: 0.390734 */ 1, 5, 6, 4,
    /* mean: 0.323144 var: 0.219202 maxCorr: 0.392163 */ -1, -1, 7, 11,
    /* mean: 0.312227 var: 0.215211 maxCorr: 0.398308 */ -11, -10, 13, 1,
    /* mean: 0.310044 var: 0.214385 maxCorr: 0.387201 */ 2, -7, 6, -2,
    /* mean: 0.30131 var: 0.210983 maxCorr: 0.396971 */ -4, -1, -2, 4,
    /* mean: 0.292576 var: 0.207429 maxCorr: 0.398003 */ -11, 7, 13, -3,
    /* mean: 0.277293 var: 0.20084 maxCorr: 0.399452 */ -3, 10, 2, -2,
    /* mean: 0.272926 var: 0.198871 maxCorr: 0.399695 */ -9, -5, 7, 7,
    /* mean: 0.270742 var: 0.197874 maxCorr: 0.398946 */ -5, -3, 2, 8,
    /* mean: 0.255459 var: 0.190616 maxCorr: 0.398223 */ -1, -6, 8, 2,
    /* mean: 0.251092 var: 0.188456 maxCorr: 0.390888 */ -8, 7, 6, 6,
    /* mean: 0.235808 var: 0.180597 maxCorr: 0.394939 */ -5, 0, 2, -8,
    /* mean: 0.218341 var: 0.171042 maxCorr: 0.394018 */ -6, -10, 2, -2,
    /* mean: 0.218341 var: 0.171042 maxCorr: 0.393413 */ -1, 0, 5, 4,
    /* mean: 0.209607 var: 0.166034 maxCorr: 0.394028 */ -2, 6, 2, 1,
    /* mean: 0.203057 var: 0.162179 maxCorr: 0.392886 */ -5, -6, 7, -3,
    /* mean: 0.170306 var: 0.14161 maxCorr: 0.374377 */ -8, 0, 5, 3,
    /* mean: 0.135371 var: 0.117302 maxCorr: 0.391212 */ -2, -3, 3, -4,
    /* mean: 0.0960699 var: 0.0870303 maxCorr: 0.398512 */ -5, 0, 5, -2,
    /* mean: 0.0895196 var: 0.0816846 maxCorr: 0.385781 */ -1, -2, 5, 1,
    /* mean: 0.069869 var: 0.0651292 maxCorr: 0.380376 */ -1, -3, 2, 2,
    /* mean: 0.0480349 var: 0.0458278 maxCorr: 0.385637 */ -5, 2, 3, 1,
    /* mean: 0.0436681 var: 0.0418528 maxCorr: 0.396232 */ -3, -2, 2, 2,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.441308 */ 1, 12, 2, -13,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.44603 */ -6, 3, -4, -2,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.449782 */ 4, -4, 9, -7,
    /* mean: 0.5 var: 0.250547 maxCorr: 0.424374 */ 7, 5, 9, -5,
    /* mean: 0.502183 var: 0.250542 maxCorr: 0.445419 */ 13, -7, 13, 7,
    /* mean: 0.502183 var: 0.250542 maxCorr: 0.446816 */ 3, -4, 5, 6,
    /* mean: 0.502183 var: 0.250542 maxCorr: 0.449961 */ -1, -8, -1, 5,
    /* mean: 0.497817 var: 0.250542 maxCorr: 0.442852 */ -13, -5, -8, -9,
    /* mean: 0.495633 var: 0.250528 maxCorr: 0.433384 */ -13, 13, -8, 10,
    /* mean: 0.495633 var: 0.250528 maxCorr: 0.449799 */ 4, 13, 13, 13,
    /* mean: 0.50655 var: 0.250503 maxCorr: 0.420805 */ 4, 3, 9, 3,
    /* mean: 0.50655 var: 0.250503 maxCorr: 0.449711 */ -6, -13, -4, 7,
    /* mean: 0.491266 var: 0.250471 maxCorr: 0.44514 */ 0, -3, 0, 4,
    /* mean: 0.4869 var: 0.250375 maxCorr: 0.43173 */ 6, -10, 11, -9,
    /* mean: 0.515284 var: 0.250313 maxCorr: 0.441125 */ 1, 4, 3, 9,
    /* mean: 0.482533 var: 0.250241 maxCorr: 0.441317 */ -10, 5, -4, 4,
    /* mean: 0.519651 var: 0.25016 maxCorr: 0.445759 */ 3, 5, 6, -10,
    /* mean: 0.465066 var: 0.249324 maxCorr: 0.438061 */ -10, -2, -8, 13,
    /* mean: 0.462882 var: 0.249166 maxCorr: 0.446648 */ -13, 2, -5, 8,
    /* mean: 0.541485 var: 0.248822 maxCorr: 0.437173 */ -13, -11, -10, -6,
    /* mean: 0.456332 var: 0.248636 maxCorr: 0.446745 */ -13, -2, -5, -6,
    /* mean: 0.456332 var: 0.248636 maxCorr: 0.442924 */ 1, -8, 5, 11,
    /* mean: 0.545852 var: 0.24844 maxCorr: 0.437339 */ -11, 12, -3, -3,
    /* mean: 0.454148 var: 0.24844 maxCorr: 0.447866 */ -4, 13, 0, -6,
    /* mean: 0.451965 var: 0.248233 maxCorr: 0.448004 */ 7, 13, 12, 0,
    /* mean: 0.445415 var: 0.247561 maxCorr: 0.44055 */ 1, -8, 13, -12,
    /* mean: 0.556769 var: 0.247318 maxCorr: 0.425494 */ -10, -4, -6, 1,
    /* mean: 0.443231 var: 0.247317 maxCorr: 0.444564 */ -8, -1, -6, -6,
    /* mean: 0.563319 var: 0.246529 maxCorr: 0.44874 */ 10, -8, 13, -13,
    /* mean: 0.430131 var: 0.245655 maxCorr: 0.439485 */ -12, 4, -3, -13,
    /* mean: 0.427948 var: 0.245344 maxCorr: 0.444954 */ 1, 9, 7, 13,
    /* mean: 0.427948 var: 0.245344 maxCorr: 0.424354 */ -7, -11, -2, -13,
    /* mean: 0.572052 var: 0.245344 maxCorr: 0.444469 */ 3, -1, 13, 2,
    /* mean: 0.421397 var: 0.244354 maxCorr: 0.448088 */ -13, -13, 0, -7,
    /* mean: 0.419214 var: 0.244006 maxCorr: 0.442495 */ 2, -4, 7, -5,
    /* mean: 0.58952 var: 0.242516 maxCorr: 0.434775 */ -5, -4, -5, 1,
    /* mean: 0.408297 var: 0.24212 maxCorr: 0.448639 */ -4, 12, 13, 10,
    /* mean: 0.408297 var: 0.24212 maxCorr: 0.449823 */ -11, -7, 0, -11,
    /* mean: 0.40393 var: 0.241298 maxCorr: 0.446521 */ 2, 10, 6, -5,
    /* mean: 0.40393 var: 0.241298 maxCorr: 0.449172 */ 2, 12, 13, 2,
    /* mean: 0.40393 var: 0.241298 maxCorr: 0.432558 */ -8, 9, -3, 13,
    /* mean: 0.598253 var: 0.240872 maxCorr: 0.438631 */ -7, -7, -4, -2,
    /* mean: 0.401747 var: 0.240872 maxCorr: 0.404411 */ -2, -8, 2, -13,
    /* mean: 0.393013 var: 0.239075 maxCorr: 0.416749 */ 0, 13, 5, 9,
    /* mean: 0.39083 var: 0.238603 maxCorr: 0.446868 */ -1, -13, 12, 10,
    /* mean: 0.620087 var: 0.236094 maxCorr: 0.438556 */ 3, -2, 5, -7,
    /* mean: 0.379913 var: 0.236094 maxCorr: 0.437419 */ -2, 4, -1, 9,
    /* mean: 0.368996 var: 0.233346 maxCorr: 0.435391 */ -2, -11, 1, -6,
    /* mean: 0.366812 var: 0.23277 maxCorr: 0.444199 */ -10, 12, 9, 13,
    /* mean: 0.362445 var: 0.231584 maxCorr: 0.443895 */ 1, -10, 9, -7,
    /* mean: 0.353712 var: 0.2291 maxCorr: 0.448392 */ -2, -1, -1, -6,
    /* mean: 0.340611 var: 0.225086 maxCorr: 0.439083 */ -7, 7, 12, -13,
    /* mean: 0.670306 var: 0.221479 maxCorr: 0.444757 */ 4, -1, 13, 13,
    /* mean: 0.327511 var: 0.220728 maxCorr: 0.444605 */ -6, 5, 1, 13,
    /* mean: 0.327511 var: 0.220728 maxCorr: 0.444964 */ -13, 0, 2, -6,
    /* mean: 0.325328 var: 0.219969 maxCorr: 0.448825 */ -6, 1, -1, -1,
    /* mean: 0.323144 var: 0.219202 maxCorr: 0.443209 */ -8, 12, 5, -5,
    /* mean: 0.318777 var: 0.217633 maxCorr: 0.448324 */ -4, -7, 8, -10,
    /* mean: 0.31441 var: 0.216028 maxCorr: 0.446346 */ -4, -13, 7, -5,
    /* mean: 0.310044 var: 0.214385 maxCorr: 0.443537 */ -1, 6, 4, 6,
    /* mean: 0.30786 var: 0.213549 maxCorr: 0.442924 */ -4, -6, 1, -9,
    /* mean: 0.299127 var: 0.210109 maxCorr: 0.447495 */ -2, 1, 4, -12,
    /* mean: 0.299127 var: 0.210109 maxCorr: 0.442517 */ -3, 5, 11, 7,
    /* mean: 0.29476 var: 0.208331 maxCorr: 0.442204 */ 2, -10, 3, 1,
    /* mean: 0.286026 var: 0.204662 maxCorr: 0.447174 */ -1, -5, 1, 4,
    /* mean: 0.286026 var: 0.204662 maxCorr: 0.436999 */ 1, 8, 4, 2,
    /* mean: 0.283843 var: 0.20372 maxCorr: 0.442079 */ -4, 2, -1, 7,
    /* mean: 0.275109 var: 0.19986 maxCorr: 0.449815 */ -13, -6, 1, 1,
    /* mean: 0.272926 var: 0.198871 maxCorr: 0.44603 */ -1, 1, 13, -3,
    /* mean: 0.272926 var: 0.198871 maxCorr: 0.449272 */ -1, -1, 6, -6,
    /* mean: 0.251092 var: 0.188456 maxCorr: 0.448119 */ -13, 3, 1, 2,
    /* mean: 0.224891 var: 0.174696 maxCorr: 0.433825 */ -3, 2, -1, -3,
    /* mean: 0.213974 var: 0.168556 maxCorr: 0.44892 */ -7, -3, 1, 4,
    /* mean: 0.203057 var: 0.162179 maxCorr: 0.446809 */ -4, 5, 1, 4,
};


} //namespace FeatureHandling
} //namespace VOCPP