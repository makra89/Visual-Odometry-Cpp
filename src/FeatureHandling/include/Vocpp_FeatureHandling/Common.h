/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_FEATURE_HANDLING_COMMON_H
#define VOCPP_FEATURE_HANDLING_COMMON_H

#include<bitset>
#include<opencv2/core/types.hpp>
#include<opencv2/core/core.hpp>
#include<iostream>
namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief Struct for storing a feature under a certain ID
  * The 2D image coordinate system is defined as follows: 
  *     - Value range of X is 0 : width of image
  *     - Value range of Y is 0 : height of image
  */
struct Feature
{
    int id; ///< ID should assigned by the feature detector
    int frameId; ///< ID of frame this feature has been detected in
    float imageCoordX; ///< image coordinates along image X axis [pixel]
    float imageCoordY; ///< image coordinates along image Y axis [pixel]
    float response; ///< "Goodness" of feature
    float angle; ///< orientation of feature [rad]

    /**
      * /brief Compare two features according to their response
      */
    bool operator > (const Feature& right) const
    {
        return response > right.response;
    }
};

/**
  * /brief Class for storing a binary feature description of a certain length
  * together with the described feature
  */
class BinaryFeatureDescription
{
public:
    
    /**
      * /brief Constructor
      */
    BinaryFeatureDescription(Feature in_feature, std::vector<bool> in_description) :
        m_feature(in_feature),
        m_description(in_description)
    {
        m_description = in_description;
        m_feature = in_feature;
    }

    const std::vector<bool>& GetDescription() const
    {
        return m_description;
    }

    const Feature& GetFeature() const
    {
        return m_feature;
    }
    
private:
    Feature m_feature; ///< described feature
    std::vector<bool> m_description; ///< feature description
};

/**
  * /brief Struct for storing a match of two binary feature descriptions
  */
class BinaryDescriptionMatch
{
public:
    BinaryDescriptionMatch(const BinaryFeatureDescription& in_first, const BinaryFeatureDescription& in_second, const float& in_distance) :
        m_descriptionFirst(in_first),
        m_descriptionSecond(in_second),
        m_distance(in_distance)
    {
    }

    const BinaryFeatureDescription& GetFirstDescription() const
    {
        return m_descriptionFirst;
    }

    const BinaryFeatureDescription& GetSecondDescription() const
    {
        return m_descriptionSecond;
    }

    const Feature& GetFirstFeature() const
    {
        return m_descriptionFirst.GetFeature();
    }

    const Feature& GetSecondFeature() const
    {
        return m_descriptionSecond.GetFeature();
    }

private:
    BinaryFeatureDescription m_descriptionFirst;
    BinaryFeatureDescription m_descriptionSecond;
    float m_distance; ///< distance measure for the two descriptions
};

/**
* /brief Get image coordinates of matching features, it is assumed that in_matches contains only
* matches for two individual frames!
*
* \param[in] in_matches vector of containing matching features
* \param[out] out_firstPoints image coordinates of "first" features
* \param[out] out_secondPoints image coordinates of "second" features
*/
static void GetMatchingPoints(const std::vector<BinaryDescriptionMatch>& in_matches, std::vector<cv::Point2f>& out_firstPoints, 
    std::vector<cv::Point2f>& out_secondPoints)
{
    for (auto match : in_matches)
    {
        cv::Point2f pointFirst(match.GetFirstFeature().imageCoordX, match.GetFirstFeature().imageCoordY);
        cv::Point2f pointSecond(match.GetSecondFeature().imageCoordX, match.GetSecondFeature().imageCoordY);
        out_firstPoints.push_back(pointFirst);
        out_secondPoints.push_back(pointSecond);
    }
}

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_FEATURE_HANDLING_COMMON_H */
