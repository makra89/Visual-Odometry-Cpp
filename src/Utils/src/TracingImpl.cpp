/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2021 Manuel Kraus
*/

#include <Vocpp_Utils/TracingImpl.h>

namespace VOCPP
{

static TraceLevel::Enum s_traceLevel = TraceLevel::TL_INFO;
static Tracer* s_tracer = nullptr;

void SetTraceLevel(const TraceLevel::Enum& in_traceLevel)
{
    s_traceLevel = in_traceLevel;
}

bool RegisterTracer(Tracer* in_tracerImpl)
{
    bool result = false;
    
    if (s_tracer == nullptr || in_tracerImpl == nullptr)
    {
        s_tracer = in_tracerImpl;
    }

    return result;
}

namespace Utils
{

Tracer* GetTracerImpl()
{
    return s_tracer;
}

bool TraceLevelActive(const TraceLevel::Enum& in_traceLevel)
{
    return in_traceLevel >= s_traceLevel;
}


} //namespace Utils
} //namespace VOCPP
