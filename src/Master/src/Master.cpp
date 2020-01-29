/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_Master/Master.h>
#include<iostream>



namespace VOCPP
{
namespace Master
{

Master::Master()
{
    // Intantiate reconstructor
    m_reconstructor = DeltaPoseReconstruction::DeltaPoseReconstructor();
}

bool Master::FeedNextFrame(Utils::Frame& in_frame)
{  
    
    // Measure frame processing time
    cv::TickMeter tick;
    tick.start();

    bool ret = m_reconstructor.FeedNextFrame(in_frame);

    tick.stop();
    std::cout << "[Master]: Frame processing time: " << tick.getTimeMilli() << std::endl;

    return ret;
}

} //namespace Master
} //namespace VOCPP
