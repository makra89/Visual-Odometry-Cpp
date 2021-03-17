/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file 
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_BRIEF_DESCRIPTOR_H
#define VOCPP_BRIEF_DESCRIPTOR_H

#include <Vocpp_Interface/Frame.h>
#include <Vocpp_FeatureHandling/Common.h>
#include <fstream>
#include <numeric>

namespace VOCPP
{
namespace FeatureHandling
{

class TrainPosition;

/**
  * /brief BRIEF descriptor, computes binary feature description
  * In order to achieve the best performance of the rotated Brief descriptor the
  * test pattern has to be learned. If the training mode is activated, the provided features
  * will be used to learn the best pattern. The pattern will be written to a file after the
  * specified number of frames has been processed
  */
class BriefDescriptor
{
public:

    /**
      * /brief Number of pairs drawn, also specifies number of bits in the returned binary description
      */
    constexpr static uint32_t s_numRandomPairs = BinaryFeatureDescription::GetSizeInBytes() * 8U;

    /**
      * \brief Constructor
      *
      * \param[in] in_randomPairDrawRadius radius around each feature pixel in which the random pairs are located
      * \param[in] in_areaDetRadius radius around each individual position in a pair used for determining the area (for intensity comparison)
      * \param[in] in_trainingMode specifies whether training mode shall be activated (see class description for explanation)
      */
    BriefDescriptor(const int& in_randomPairDrawRadius = 15, const int& in_areaDetRadius = 2, 
        const bool& in_trainingMode = false, std::string in_filePath = "Brief_TrainedPattern.txt", const unsigned int& in_numFramesForTraining = 1000U);

    /**
      * /brief Compute binary feature descriptions for provided frame. The returned description IDs will
      * be based on the provided feature IDs. Note that it might be the case that not for all features a valid
      * description can be computed
      *
      * \param[in] in_frame frame out of which the features have been extracted
      * \param[in] in_features features extracted from frame
      * \param[out] out_descriptions binary descriptions computed for provided features
      *
      * \return True if feature description for at least one keypoint successful, false otherwise
      */
    bool ComputeDescriptions(const Frame& in_frame, const std::vector<Feature>& in_features,
        std::vector<BinaryFeatureDescription>& out_descriptions);

    /**
      * /brief Struct for storing point pairs drawn by the BRIEF descriptor
      */
    struct PointPair
    {
        double x1;
        double y1;
        double x2;
        double y2;
    };

private:
    // It is not allowed to copy the descriptor directly
    BriefDescriptor& operator=(const BriefDescriptor&);
    BriefDescriptor(const BriefDescriptor&);

    /**
      * /brief Fill m_pairs with the test positions
      */
    void GetTestPositions();

    /**
      * /brief Rotate a point pair using a (feature) angle
      */
    PointPair RotatePair(const double& in_angle, const PointPair& in_pair);

    int m_randomPairDrawRadius; ///< radius around a feature that is used for drawing the random pairs
    int m_areaDetRadius; ///< radius around point in a pair used for area determination
    std::vector<PointPair> m_pairs; ///< Point pairs used for intensity comparison
    static const int s_testPattern[256*4];
    /// ============================================================================
    ///             The following members belong to the training mode                
    /// ============================================================================
    bool m_trainingMode; ///< specifies whether training is switched on or off
    std::ofstream m_patternOutFile; ///< file the trained pattern is written to
    unsigned int m_numFramesForTraining; ///< number of processed frames after which the best pattern will be written

     /**
      * /brief training position class, each position specify a possible test pattern
      */
    class TrainPosition
    {
    public:
        /**
          * /brief Constructor
          */
        TrainPosition(const unsigned int& in_Id, const BriefDescriptor::PointPair& in_pair) :
            m_Id(in_Id),
            m_testResults(new std::vector<bool>()),
            m_pair(in_pair)
        {
        }
        
        /**
          * /brief Add a test result to the storage
          */
        void AddVal(const bool& in_result)
        {
            m_testResults->push_back(static_cast<int>(in_result));
        }

        /**
          * /brief Calculate Sample Mean (Expected Value: 0.5)
          */
        double GetMean() const
        {
            double sum = std::accumulate(m_testResults->begin(), m_testResults->end(), 0.0F);
            return sum / static_cast<double>(m_testResults->size());
        }

        /**
          * /brief Calculate Sample Variance (Expected Value: 0.25)
          */
        double GetVariance() const
        {
            double variance = 0.0;
            double mean = GetMean();
            for (auto val : *m_testResults)
            {
                variance += std::pow(val - mean, 2);
            }

            return variance / (static_cast<double>(m_testResults->size()) - 1.0F);
        }

        /**
          * /brief Operator > used for sorting a vector of TrainPositions
          */
        bool operator > (const TrainPosition& right) const
        {
            return std::abs(0.5 - GetMean()) > std::abs(0.5 - right.GetMean());
            return m_Id > right.m_Id;
        }

        /**
          * /brief Copy assignment
          */
        TrainPosition& operator=(TrainPosition right)
        {
            m_Id = right.m_Id;
            m_testResults = right.m_testResults;
            m_pair = right.m_pair;
            return *this;
        }

        unsigned int m_Id; ///< Id of TrainPosition
        std::shared_ptr<std::vector<bool>> m_testResults; ///< vector of boolean test results (result of intensity comparison)
        BriefDescriptor::PointPair m_pair; ///< Test pattern used
    };

    /**
      * \brief Draw all possible test positions for training (Will fill m_pair)
      */
    void DrawTrainPairs();

    /**
      * \brief Calculate Pearson Correlation between two TrainPositions
      *
      * \return correlation score between [-1,1] if sucessful, otherwise -999.0
      */
    double CalculatePearsonCorr(const TrainPosition& in_first, const TrainPosition& in_second);

    /**
      * \brief Find best pattern using stored train positions (test vectors must be filled) and writes them to a file
      */
    void FindBestPattern(const std::vector<TrainPosition>& in_trainPositions, std::ofstream& in_file);

    std::vector<TrainPosition> m_trainPositions; ///< test positions used for training
};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_BRIEF_DESCRIPTOR_H */
