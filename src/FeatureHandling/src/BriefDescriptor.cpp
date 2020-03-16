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
#include <map>

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
            m_trainingMode = false;
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
            denomFirst += std::pow((*in_first.m_testResults)[i] - firstMean, 2);
            denomSec += std::pow((*in_second.m_testResults)[i] - secMean, 2);
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
    
    // Do some preselection based on distance of mean to 0.5
    std::vector<TrainPosition> preselect;
    for (auto pos : copy)
    {
        if (std::abs(pos.GetMean() - 0.5) < 0.1)
        {
            preselect.push_back(pos);
        }
    }

    // Copy first position to best pattern vector (we have to start somewhere)
    std::vector<TrainPosition> bestPatterns;
    bestPatterns.push_back(preselect[0]);
    in_file << "/* mean: " << preselect[0].GetMean() << " var: " << preselect[0].GetVariance() << " maxCorr: " << 0.0 << " */ " <<
        preselect[0].m_pair.x1 << ", " << preselect[0].m_pair.y1 << ", " << preselect[0].m_pair.x2 << ", " << preselect[0].m_pair.y2 << ", " << std::endl;

    float thresh = 0.1F;
    std::map<std::pair<int, int>, float> corrMap;
    while (bestPatterns.size() < BriefDescriptor::s_numRandomPairs)
    {
        for (const auto pos : preselect)
        {
            if (bestPatterns.size() >= BriefDescriptor::s_numRandomPairs)
            {
                break;
            }
            
            // Find test position with lowest correlation to existing "best positions"
            float maximumCorr = 0.0;
            for (const auto bestPos : bestPatterns)
            {
                float corr = 1.0;
                
                if (corrMap.count(std::make_pair(pos.m_Id, bestPos.m_Id)) > 0)
                {
                    corr = corrMap[std::make_pair(pos.m_Id, bestPos.m_Id)];
                }
                else if(corrMap.count(std::make_pair(bestPos.m_Id, pos.m_Id)) > 0)
                {
                    corr = corrMap[std::make_pair(bestPos.m_Id, pos.m_Id)];
                }
                else
                {
                    corr = CalculatePearsonCorr(pos, bestPos);
                    corrMap[std::make_pair(bestPos.m_Id, pos.m_Id)] = corr;
                    corrMap[std::make_pair(pos.m_Id, bestPos.m_Id)] = corr;
                }

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
Trained with PASCAL 2006 image set (like in the original ORB paper) using 300k features */
const int BriefDescriptor::s_testPattern[256 * 4] =
{
    /* mean: 0.500001 var: 0.250001 maxCorr: 0.0000000 */ 10, 9, -5, 1,
    /* mean: 0.500002 var: 0.250001 maxCorr: 0.0863017 */ 10, 9, 11, -9,
    /* mean: 0.499988 var: 0.250001 maxCorr: 0.0737321 */ -12, 0, -3, -4,
    /* mean: 0.500021 var: 0.250001 maxCorr: 0.0606839 */ 2, 3, 12, 3,
    /* mean: 0.500429 var: 0.250001 maxCorr: 0.0706252 */ 0, -3, 0, 3,
    /* mean: 0.50149 var: 0.249997 maxCorr: 0.0999704 */ 0, -12, 0, -7,
    /* mean: 0.497735 var: 0.249992 maxCorr: 0.0902379 */ 7, -10, 12, -13,
    /* mean: 0.515524 var: 0.249555 maxCorr: 0.0991998 */ -2, 6, -1, 1,
    /* mean: 0.516454 var: 0.249813 maxCorr: 0.0978358 */ 8, 9, 13, 13,
    /* mean: 0.476131 var: 0.24932 maxCorr: 0.0971741 */ -1, 13, 0, 8,
    /* mean: 0.469527 var: 0.248712 maxCorr: 0.096573 */ -13, 8, -13, 13,
    /* mean: 0.530746 var: 0.248634 maxCorr: 0.077915 */ -13, -13, -12, -8,
    /* mean: 0.427082 var: 0.244785 maxCorr: 0.0951481 */ -4, 13, 1, 13,
    /* mean: 0.422961 var: 0.244088 maxCorr: 0.0865895 */ -3, -13, 2, -13,
    /* mean: 0.505527 var: 0.249946 maxCorr: 0.140948 */ -7, -2, -4, 3,
    /* mean: 0.51229 var: 0.249795 maxCorr: 0.139875 */ 3, -2, 4, 3,
    /* mean: 0.48541 var: 0.2499 maxCorr: 0.130444 */ 0, -8, 0, -3,
    /* mean: 0.477309 var: 0.24943 maxCorr: 0.13619 */ -13, -12, -8, -13,
    /* mean: 0.469616 var: 0.248734 maxCorr: 0.14354 */ 11, -12, 13, -7,
    /* mean: 0.447316 var: 0.246691 maxCorr: 0.145622 */ 8, 12, 9, 7,
    /* mean: 0.439985 var: 0.24661 maxCorr: 0.149439 */ -8, 10, -3, 11,
    /* mean: 0.499322 var: 0.249999 maxCorr: 0.19149 */ -13, 8, -8, 8,
    /* mean: 0.493389 var: 0.249935 maxCorr: 0.195348 */ -10, -12, -5, -9,
    /* mean: 0.529525 var: 0.24958 maxCorr: 0.189421 */ 8, 3, 13, -1,
    /* mean: 0.531079 var: 0.248679 maxCorr: 0.180747 */ -13, -3, -9, 2,
    /* mean: 0.457774 var: 0.248372 maxCorr: 0.190729 */ 5, 12, 10, 11,
    /* mean: 0.455779 var: 0.248003 maxCorr: 0.170147 */ 2, -9, 7, -13,
    /* mean: 0.450048 var: 0.247468 maxCorr: 0.194247 */ 5, -13, 10, -10,
    /* mean: 0.4468 var: 0.246818 maxCorr: 0.195494 */ -9, 4, -8, 9,
    /* mean: 0.445904 var: 0.247442 maxCorr: 0.198646 */ 7, -7, 9, -2,
    /* mean: 0.435728 var: 0.246393 maxCorr: 0.184382 */ 1, 12, 6, 13,
    /* mean: 0.567924 var: 0.245903 maxCorr: 0.195564 */ -7, -10, -7, -5,
    /* mean: 0.412386 var: 0.242517 maxCorr: 0.186698 */ 5, 6, 7, 1,
    /* mean: 0.402643 var: 0.240796 maxCorr: 0.199526 */ -1, -5, 2, 9,
    /* mean: 0.501241 var: 0.25 maxCorr: 0.243818 */ 5, -5, 10, -5,
    /* mean: 0.519974 var: 0.249359 maxCorr: 0.244509 */ -7, -8, -2, -3,
    /* mean: 0.520293 var: 0.24962 maxCorr: 0.240998 */ -9, 13, -5, 8,
    /* mean: 0.520668 var: 0.249608 maxCorr: 0.246817 */ 2, 1, 5, -4,
    /* mean: 0.529783 var: 0.2495 maxCorr: 0.240745 */ -13, 3, -9, -2,
    /* mean: 0.531675 var: 0.248603 maxCorr: 0.234917 */ 10, -1, 13, 4,
    /* mean: 0.533031 var: 0.249206 maxCorr: 0.23806 */ 12, 4, 13, 9,
    /* mean: 0.542545 var: 0.248282 maxCorr: 0.246398 */ -4, 9, -2, 4,
    /* mean: 0.558954 var: 0.246837 maxCorr: 0.238119 */ -2, -4, -2, 2,
    /* mean: 0.439905 var: 0.246596 maxCorr: 0.235707 */ -7, -12, -2, -13,
    /* mean: 0.560658 var: 0.24579 maxCorr: 0.247815 */ 8, -3, 13, -7,
    /* mean: 0.401835 var: 0.240335 maxCorr: 0.225354 */ 5, -10, 5, -5,
    /* mean: 0.499805 var: 0.250001 maxCorr: 0.270208 */ -13, -6, -8, -7,
    /* mean: 0.500406 var: 0.250001 maxCorr: 0.298246 */ 9, -5, 13, 0,
    /* mean: 0.499514 var: 0.250001 maxCorr: 0.295474 */ 2, 6, 5, 12,
    /* mean: 0.500767 var: 0.25 maxCorr: 0.292468 */ -5, -13, -3, 9,
    /* mean: 0.49789 var: 0.249993 maxCorr: 0.285089 */ -10, 6, -4, 5,
    /* mean: 0.50326 var: 0.249978 maxCorr: 0.298942 */ -7, 4, -5, -4,
    /* mean: 0.495074 var: 0.249955 maxCorr: 0.29353 */ 2, -4, 7, -8,
    /* mean: 0.487926 var: 0.249819 maxCorr: 0.28873 */ -13, 1, -11, 6,
    /* mean: 0.51582 var: 0.249543 maxCorr: 0.295335 */ -13, -5, -13, 0,
    /* mean: 0.47866 var: 0.249981 maxCorr: 0.294732 */ -12, 13, -7, 13,
    /* mean: 0.526747 var: 0.249048 maxCorr: 0.298523 */ -13, -9, -13, -4,
    /* mean: 0.472366 var: 0.248957 maxCorr: 0.285738 */ 8, 10, 13, 7,
    /* mean: 0.470034 var: 0.249534 maxCorr: 0.287342 */ 13, -9, 13, -4,
    /* mean: 0.44473 var: 0.247288 maxCorr: 0.291189 */ -8, -8, -6, -13,
    /* mean: 0.442764 var: 0.246148 maxCorr: 0.289821 */ -9, 7, -7, 13,
    /* mean: 0.559733 var: 0.246398 maxCorr: 0.291229 */ 6, 4, 11, 8,
    /* mean: 0.440023 var: 0.246625 maxCorr: 0.297599 */ 0, -12, 13, 13,
    /* mean: 0.436573 var: 0.246216 maxCorr: 0.287483 */ -8, -2, -6, -7,
    /* mean: 0.424585 var: 0.244376 maxCorr: 0.299056 */ -9, 13, 3, -13,
    /* mean: 0.422698 var: 0.243966 maxCorr: 0.291394 */ 3, 13, 5, 8,
    /* mean: 0.421473 var: 0.243583 maxCorr: 0.299465 */ 2, 12, 12, -4,
    /* mean: 0.419333 var: 0.243352 maxCorr: 0.292186 */ 0, 0, 7, 8,
    /* mean: 0.417704 var: 0.243615 maxCorr: 0.292383 */ 0, -13, 5, -11,
    /* mean: 0.413583 var: 0.242794 maxCorr: 0.296777 */ -12, -3, -1, 13,
    /* mean: 0.40513 var: 0.241131 maxCorr: 0.298204 */ -6, 1, -6, 6,
    /* mean: 0.499495 var: 0.25 maxCorr: 0.339453 */ 3, 6, 5, -8,
    /* mean: 0.501115 var: 0.250001 maxCorr: 0.344583 */ 5, -5, 9, 5,
    /* mean: 0.498069 var: 0.249996 maxCorr: 0.334935 */ -13, 13, -1, 1,
    /* mean: 0.509756 var: 0.250565 maxCorr: 0.314421 */ 12, 2, 13, -3,
    /* mean: 0.485954 var: 0.249701 maxCorr: 0.349144 */ -5, -12, -2, -7,
    /* mean: 0.517284 var: 0.250261 maxCorr: 0.346981 */ 5, 3, 8, -2,
    /* mean: 0.518308 var: 0.249728 maxCorr: 0.335144 */ 13, 1, 13, 6,
    /* mean: 0.477281 var: 0.249428 maxCorr: 0.31888 */ -2, -2, -2, 3,
    /* mean: 0.477046 var: 0.249397 maxCorr: 0.339991 */ -7, 2, -2, 1,
    /* mean: 0.525489 var: 0.249709 maxCorr: 0.347629 */ -12, 3, -7, 4,
    /* mean: 0.471878 var: 0.248998 maxCorr: 0.334785 */ -13, 4, -13, 9,
    /* mean: 0.531328 var: 0.248628 maxCorr: 0.336054 */ 13, 7, 13, 12,
    /* mean: 0.542888 var: 0.247517 maxCorr: 0.339971 */ -13, 7, -10, 2,
    /* mean: 0.545601 var: 0.247865 maxCorr: 0.347468 */ -11, -7, -5, -4,
    /* mean: 0.545953 var: 0.248085 maxCorr: 0.343444 */ 8, -8, 9, -13,
    /* mean: 0.451874 var: 0.247728 maxCorr: 0.347929 */ -11, -6, -5, -10,
    /* mean: 0.44864 var: 0.247912 maxCorr: 0.349996 */ 3, 7, 8, 8,
    /* mean: 0.556293 var: 0.246343 maxCorr: 0.342166 */ -13, -13, -4, 4,
    /* mean: 0.440332 var: 0.246472 maxCorr: 0.335009 */ 4, -13, 4, -8,
    /* mean: 0.564901 var: 0.245559 maxCorr: 0.341592 */ -8, -5, -7, 0,
    /* mean: 0.432348 var: 0.245317 maxCorr: 0.348581 */ 0, 6, 13, -13,
    /* mean: 0.571777 var: 0.245073 maxCorr: 0.328076 */ 7, 0, 8, -5,
    /* mean: 0.424651 var: 0.244131 maxCorr: 0.345502 */ 2, -3, 7, 1,
    /* mean: 0.581831 var: 0.243795 maxCorr: 0.342223 */ 8, -1, 10, 13,
    /* mean: 0.416446 var: 0.242608 maxCorr: 0.349509 */ -6, 9, -1, 7,
    /* mean: 0.413968 var: 0.242864 maxCorr: 0.3486 */ -7, -7, -2, -8,
    /* mean: 0.406534 var: 0.240955 maxCorr: 0.338261 */ 3, -9, 8, -5,
    /* mean: 0.406196 var: 0.240789 maxCorr: 0.329197 */ -4, 6, -3, 11,
    /* mean: 0.500185 var: 0.250001 maxCorr: 0.396054 */ 1, -4, 1, 4,
    /* mean: 0.499308 var: 0.250001 maxCorr: 0.399883 */ 6, -13, 6, 13,
    /* mean: 0.499176 var: 0.250002 maxCorr: 0.399493 */ 0, 5, 0, 11,
    /* mean: 0.497562 var: 0.250065 maxCorr: 0.39017 */ 4, 8, 9, 13,
    /* mean: 0.504461 var: 0.249966 maxCorr: 0.397305 */ 1, 1, 13, -6,
    /* mean: 0.495511 var: 0.249965 maxCorr: 0.395056 */ -11, -6, -7, 7,
    /* mean: 0.493826 var: 0.249922 maxCorr: 0.389712 */ -13, -1, -9, -6,
    /* mean: 0.490103 var: 0.249806 maxCorr: 0.398769 */ 8, -8, 13, -7,
    /* mean: 0.514815 var: 0.249886 maxCorr: 0.399837 */ -13, -4, -2, 2,
    /* mean: 0.484598 var: 0.249558 maxCorr: 0.397173 */ -13, 6, -4, -7,
    /* mean: 0.480144 var: 0.249344 maxCorr: 0.380323 */ 0, -4, 0, 1,
    /* mean: 0.47829 var: 0.249962 maxCorr: 0.373211 */ 8, 7, 12, 2,
    /* mean: 0.471972 var: 0.249005 maxCorr: 0.396919 */ 0, 0, 5, -13,
    /* mean: 0.532623 var: 0.249227 maxCorr: 0.397772 */ 5, -6, 10, -11,
    /* mean: 0.465251 var: 0.248304 maxCorr: 0.399253 */ -4, -6, -2, 5,
    /* mean: 0.535716 var: 0.248179 maxCorr: 0.396288 */ -5, 13, -3, -6,
    /* mean: 0.46334 var: 0.248805 maxCorr: 0.396443 */ 9, -12, 13, 3,
    /* mean: 0.461594 var: 0.248594 maxCorr: 0.394309 */ -13, 2, -9, -13,
    /* mean: 0.540039 var: 0.248352 maxCorr: 0.378053 */ -13, 12, -9, 7,
    /* mean: 0.542301 var: 0.248378 maxCorr: 0.371871 */ -13, -11, -9, -6,
    /* mean: 0.452968 var: 0.24832 maxCorr: 0.397268 */ -11, -2, -11, 13,
    /* mean: 0.452531 var: 0.247538 maxCorr: 0.385403 */ -13, 5, -5, 11,
    /* mean: 0.552843 var: 0.246653 maxCorr: 0.394142 */ -11, -8, -9, -2,
    /* mean: 0.445054 var: 0.247274 maxCorr: 0.387807 */ 9, -10, 9, -5,
    /* mean: 0.554965 var: 0.247278 maxCorr: 0.390187 */ -13, -1, -8, -1,
    /* mean: 0.441947 var: 0.246698 maxCorr: 0.382425 */ 8, 9, 9, 4,
    /* mean: 0.560301 var: 0.24577 maxCorr: 0.395866 */ 6, -2, 11, -1,
    /* mean: 0.565999 var: 0.245312 maxCorr: 0.39965 */ -10, 9, -6, 4,
    /* mean: 0.43125 var: 0.245681 maxCorr: 0.39235 */ 4, 9, 10, 5,
    /* mean: 0.430466 var: 0.244797 maxCorr: 0.397709 */ -4, -8, -3, -13,
    /* mean: 0.427533 var: 0.245189 maxCorr: 0.389753 */ -4, 6, -1, -4,
    /* mean: 0.427336 var: 0.244822 maxCorr: 0.39497 */ 0, -12, 13, -13,
    /* mean: 0.421924 var: 0.244423 maxCorr: 0.396442 */ 3, -13, 13, -1,
    /* mean: 0.418812 var: 0.24333 maxCorr: 0.396501 */ -9, 2, -3, 7,
    /* mean: 0.417901 var: 0.243407 maxCorr: 0.392588 */ -1, -8, 3, -13,
    /* mean: 0.416897 var: 0.242631 maxCorr: 0.398864 */ -10, -13, 3, 12,
    /* mean: 0.413715 var: 0.242836 maxCorr: 0.396178 */ -13, -12, 1, -10,
    /* mean: 0.411218 var: 0.242037 maxCorr: 0.387815 */ -2, 13, 3, 10,
    /* mean: 0.410692 var: 0.241884 maxCorr: 0.393591 */ -3, 13, 13, 13,
    /* mean: 0.409936 var: 0.241728 maxCorr: 0.386171 */ -9, 4, -2, -13,
    /* mean: 0.409237 var: 0.241778 maxCorr: 0.393882 */ -2, 8, 2, 13,
    /* mean: 0.408449 var: 0.241914 maxCorr: 0.392291 */ -13, 12, 2, 11,
    /* mean: 0.591749 var: 0.241922 maxCorr: 0.360437 */ -3, -10, -3, -5,
    /* mean: 0.408242 var: 0.241922 maxCorr: 0.397496 */ -6, -4, -1, -1,
    /* mean: 0.500106 var: 0.250001 maxCorr: 0.44375 */ 1, -11, 1, 12,
    /* mean: 0.499406 var: 0.250003 maxCorr: 0.401703 */ 1, -1, 2, 4,
    /* mean: 0.500805 var: 0.250001 maxCorr: 0.449725 */ 2, -5, 6, 10,
    /* mean: 0.500974 var: 0.25 maxCorr: 0.448508 */ 2, -4, 13, 8,
    /* mean: 0.501331 var: 0.249998 maxCorr: 0.449687 */ 9, 4, 10, -3,
    /* mean: 0.502668 var: 0.249987 maxCorr: 0.424621 */ -6, 13, -2, 6,
    /* mean: 0.496614 var: 0.249975 maxCorr: 0.441161 */ -13, -4, -13, 5,
    /* mean: 0.504006 var: 0.24997 maxCorr: 0.444173 */ -4, -9, -1, 1,
    /* mean: 0.493901 var: 0.249941 maxCorr: 0.445527 */ -10, 2, -9, -4,
    /* mean: 0.508545 var: 0.249886 maxCorr: 0.443627 */ 6, 6, 13, -6,
    /* mean: 0.490024 var: 0.249822 maxCorr: 0.446288 */ -10, -1, -9, 4,
    /* mean: 0.514393 var: 0.249617 maxCorr: 0.447149 */ -9, 8, -2, -3,
    /* mean: 0.48281 var: 0.249542 maxCorr: 0.442438 */ 5, 8, 13, 9,
    /* mean: 0.518223 var: 0.249734 maxCorr: 0.411873 */ 8, 5, 13, 5,
    /* mean: 0.480496 var: 0.249306 maxCorr: 0.436579 */ 13, -6, 13, -1,
    /* mean: 0.47721 var: 0.249408 maxCorr: 0.441821 */ 8, -12, 13, -11,
    /* mean: 0.476656 var: 0.249068 maxCorr: 0.401027 */ 8, 13, 13, 12,
    /* mean: 0.471507 var: 0.249685 maxCorr: 0.432187 */ -13, -6, -10, -11,
    /* mean: 0.471479 var: 0.249685 maxCorr: 0.444193 */ -12, 10, -4, 9,
    /* mean: 0.470963 var: 0.249644 maxCorr: 0.449813 */ -13, -10, -5, 13,
    /* mean: 0.533332 var: 0.249466 maxCorr: 0.417257 */ 11, -8, 13, -13,
    /* mean: 0.462491 var: 0.249002 maxCorr: 0.444407 */ 3, -8, 13, -9,
    /* mean: 0.461369 var: 0.248053 maxCorr: 0.447341 */ -11, 6, -11, 11,
    /* mean: 0.459313 var: 0.248506 maxCorr: 0.422149 */ -10, -11, -5, -13,
    /* mean: 0.458342 var: 0.248409 maxCorr: 0.434136 */ 1, -3, 1, 2,
    /* mean: 0.454047 var: 0.248085 maxCorr: 0.436238 */ -1, 6, 0, -7,
    /* mean: 0.550834 var: 0.247767 maxCorr: 0.444538 */ 9, 4, 13, -13,
    /* mean: 0.551243 var: 0.247917 maxCorr: 0.436952 */ 9, 2, 10, 7,
    /* mean: 0.551402 var: 0.247913 maxCorr: 0.419834 */ 8, 6, 11, 11,
    /* mean: 0.447321 var: 0.246691 maxCorr: 0.442851 */ 6, 8, 7, -4,
    /* mean: 0.446852 var: 0.246824 maxCorr: 0.423714 */ 7, -12, 10, -7,
    /* mean: 0.555148 var: 0.247304 maxCorr: 0.44336 */ -10, 6, -8, 0,
    /* mean: 0.442623 var: 0.246779 maxCorr: 0.440351 */ 0, -5, 2, -10,
    /* mean: 0.559198 var: 0.246469 maxCorr: 0.448753 */ -7, -10, -6, 4,
    /* mean: 0.440759 var: 0.246477 maxCorr: 0.441225 */ -8, -4, -2, -4,
    /* mean: 0.559931 var: 0.246645 maxCorr: 0.446622 */ 8, 1, 13, 2,
    /* mean: 0.564216 var: 0.245768 maxCorr: 0.444433 */ 1, -2, 3, 13,
    /* mean: 0.574561 var: 0.244536 maxCorr: 0.449964 */ 2, 2, 11, 13,
    /* mean: 0.575354 var: 0.244129 maxCorr: 0.449943 */ 8, -1, 10, -9,
    /* mean: 0.424125 var: 0.244281 maxCorr: 0.441313 */ 3, -11, 7, 6,
    /* mean: 0.576842 var: 0.244119 maxCorr: 0.433379 */ -9, -13, -4, -4,
    /* mean: 0.422895 var: 0.244335 maxCorr: 0.447504 */ -2, 13, 9, -12,
    /* mean: 0.421107 var: 0.243772 maxCorr: 0.442627 */ -8, -2, -5, 10,
    /* mean: 0.420722 var: 0.243703 maxCorr: 0.434653 */ 2, -10, 7, -9,
    /* mean: 0.420276 var: 0.243268 maxCorr: 0.447199 */ -1, -13, 1, 4,
    /* mean: 0.42 var: 0.243223 maxCorr: 0.443952 */ 1, -5, 13, -2,
    /* mean: 0.417728 var: 0.243617 maxCorr: 0.44542 */ -6, 1, -3, -4,
    /* mean: 0.583708 var: 0.242513 maxCorr: 0.441074 */ -10, 3, -5, -1,
    /* mean: 0.412987 var: 0.242397 maxCorr: 0.449796 */ 1, 4, 8, -5,
    /* mean: 0.4128 var: 0.242529 maxCorr: 0.44458 */ -6, 9, -1, 13,
    /* mean: 0.410758 var: 0.241916 maxCorr: 0.441855 */ 0, 3, 2, -6,
    /* mean: 0.407806 var: 0.242013 maxCorr: 0.449607 */ 1, 12, 9, 8,
    /* mean: 0.407491 var: 0.241951 maxCorr: 0.432506 */ -2, -13, 2, -8,
    /* mean: 0.407223 var: 0.24114 maxCorr: 0.446414 */ -11, -9, 0, 5,
    /* mean: 0.59288 var: 0.2411 maxCorr: 0.440782 */ 6, 0, 7, 5,
    /* mean: 0.406881 var: 0.241056 maxCorr: 0.44695 */ -5, -10, 0, -10,
    /* mean: 0.406304 var: 0.240925 maxCorr: 0.444166 */ -13, -3, 0, -13,
    /* mean: 0.406064 var: 0.240765 maxCorr: 0.420786 */ 5, 10, 6, 5,
    /* mean: 0.595785 var: 0.241187 maxCorr: 0.446755 */ 4, -1, 9, -2,
    /* mean: 0.402929 var: 0.240828 maxCorr: 0.439983 */ 0, 8, 6, 10,
    /* mean: 0.40184 var: 0.240334 maxCorr: 0.449676 */ -13, 10, 1, -7,
    /* mean: 0.598634 var: 0.239925 maxCorr: 0.446892 */ 4, 3, 9, -12,
    /* mean: 0.400375 var: 0.239935 maxCorr: 0.42328 */ 4, -4, 5, 1,
    /* mean: 0.500101 var: 0.250001 maxCorr: 0.478011 */ -12, -1, -5, 5,
    /* mean: 0.499829 var: 0.250001 maxCorr: 0.494488 */ -9, 13, -6, -11,
    /* mean: 0.499777 var: 0.250001 maxCorr: 0.483998 */ -13, -10, -8, -9,
    /* mean: 0.500368 var: 0.25 maxCorr: 0.476018 */ -3, 13, -1, 0,
    /* mean: 0.49951 var: 0.250001 maxCorr: 0.496204 */ 8, -7, 13, 6,
    /* mean: 0.500547 var: 0.250001 maxCorr: 0.498748 */ 5, -8, 10, 11,
    /* mean: 0.498928 var: 0.249998 maxCorr: 0.496834 */ -13, 3, -2, 3,
    /* mean: 0.502434 var: 0.249986 maxCorr: 0.492414 */ 2, -6, 5, -12,
    /* mean: 0.49636 var: 0.249973 maxCorr: 0.470625 */ -5, 4, -2, -1,
    /* mean: 0.493868 var: 0.249941 maxCorr: 0.495695 */ 4, 5, 13, 0,
    /* mean: 0.49368 var: 0.249996 maxCorr: 0.497388 */ 6, -6, 13, -3,
    /* mean: 0.493075 var: 0.249902 maxCorr: 0.49113 */ -7, -13, -1, 3,
    /* mean: 0.49024 var: 0.250565 maxCorr: 0.497538 */ 6, 6, 11, 5,
    /* mean: 0.510488 var: 0.249797 maxCorr: 0.497695 */ 8, -3, 10, 2,
    /* mean: 0.487602 var: 0.249734 maxCorr: 0.49771 */ 10, 7, 13, -2,
    /* mean: 0.51253 var: 0.249729 maxCorr: 0.492338 */ 0, -3, 0, 5,
    /* mean: 0.514585 var: 0.249897 maxCorr: 0.499779 */ -10, 8, -7, -6,
    /* mean: 0.484316 var: 0.249548 maxCorr: 0.49627 */ -11, -3, -6, -6,
    /* mean: 0.480693 var: 0.249373 maxCorr: 0.499875 */ 2, 3, 3, -4,
    /* mean: 0.480641 var: 0.249372 maxCorr: 0.498796 */ -12, 2, -8, 7,
    /* mean: 0.480083 var: 0.249359 maxCorr: 0.499912 */ 3, 3, 8, 0,
    /* mean: 0.522874 var: 0.249402 maxCorr: 0.489446 */ 8, -7, 13, -10,
    /* mean: 0.524855 var: 0.249971 maxCorr: 0.485131 */ -13, -9, -11, 4,
    /* mean: 0.525559 var: 0.249625 maxCorr: 0.495147 */ 11, -4, 13, 9,
    /* mean: 0.470357 var: 0.249583 maxCorr: 0.497515 */ 11, 10, 13, 5,
    /* mean: 0.469874 var: 0.249541 maxCorr: 0.493057 */ 10, -8, 12, -2,
    /* mean: 0.469559 var: 0.248732 maxCorr: 0.494514 */ 4, -5, 10, 0,
    /* mean: 0.469405 var: 0.248724 maxCorr: 0.492524 */ -13, 9, -8, 12,
    /* mean: 0.466743 var: 0.249453 maxCorr: 0.493402 */ -9, -7, -4, -7,
    /* mean: 0.533294 var: 0.249451 maxCorr: 0.477611 */ 10, 0, 11, -5,
    /* mean: 0.466189 var: 0.249178 maxCorr: 0.478619 */ 4, -11, 9, -13,
    /* mean: 0.46304 var: 0.249041 maxCorr: 0.491503 */ -12, -8, -8, -13,
    /* mean: 0.461792 var: 0.248608 maxCorr: 0.487085 */ -11, -3, -9, -9,
    /* mean: 0.461608 var: 0.248596 maxCorr: 0.498263 */ 0, 6, 3, -12,
    /* mean: 0.460449 var: 0.248647 maxCorr: 0.490559 */ 2, 4, 7, 6,
    /* mean: 0.539626 var: 0.248643 maxCorr: 0.48821 */ 13, -1, 13, 13,
    /* mean: 0.460271 var: 0.248328 maxCorr: 0.495011 */ -13, 3, -9, 12,
    /* mean: 0.540734 var: 0.248507 maxCorr: 0.491422 */ -10, -12, -10, -7,
    /* mean: 0.459163 var: 0.248567 maxCorr: 0.499468 */ -2, 10, 0, -5,
    /* mean: 0.456976 var: 0.247505 maxCorr: 0.471392 */ -7, 13, -2, 10,
    /* mean: 0.543146 var: 0.248155 maxCorr: 0.495414 */ 1, -2, 1, 3,
    /* mean: 0.543531 var: 0.248117 maxCorr: 0.495906 */ 4, 5, 8, 10,
    /* mean: 0.545014 var: 0.247917 maxCorr: 0.482098 */ 2, 4, 4, 9,
    /* mean: 0.454714 var: 0.247889 maxCorr: 0.489257 */ 6, -9, 11, -5,
    /* mean: 0.454319 var: 0.247854 maxCorr: 0.495169 */ -7, 7, -3, -8,
    /* mean: 0.453719 var: 0.247801 maxCorr: 0.499017 */ -8, -12, -2, -9,
    /* mean: 0.548502 var: 0.247355 maxCorr: 0.484104 */ -13, -13, -11, -1,
    /* mean: 0.451414 var: 0.247349 maxCorr: 0.491093 */ 8, 13, 13, 1,
    /* mean: 0.549037 var: 0.247286 maxCorr: 0.480005 */ -13, -5, -8, -3,
    /* mean: 0.549314 var: 0.247274 maxCorr: 0.468782 */ 1, 2, 2, 7
};


} //namespace FeatureHandling
} //namespace VOCPP