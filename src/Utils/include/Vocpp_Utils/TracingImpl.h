/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2021 Manuel Kraus
*/

#ifndef VOCPP_TRACING_IMPL_H
#define VOCPP_TRACING_IMPL_H

#include <Vocpp_Interface/Tracing.h>
#include <sstream>

namespace VOCPP
{

extern void SetTraceLevel(const TraceLevel& in_traceLevel);
bool RegisterTracer(Tracer* in_tracerImpl);

namespace Utils
{

Tracer* GetTracerImpl();

bool TraceLevelActive(const TraceLevel& in_traceLevel);

#define VOCPP_TRACE(in_level, in_msg) \
    if(VOCPP::Utils::TraceLevelActive(in_level) && VOCPP::Utils::GetTracerImpl() != nullptr) \
    { \
        std::stringstream stream; \
        stream << in_msg; \
        VOCPP::Utils::GetTracerImpl()->receiveTrace(in_level, stream.str().c_str()); \
    }

#define VOCPP_TRACE_INFO(in_msg) VOCPP_TRACE(VOCPP::TraceLevel::INFO, in_msg)
#define VOCPP_TRACE_WARNING(in_msg) VOCPP_TRACE(VOCPP::TraceLevel::WARNING, in_msg)
#define VOCPP_TRACE_ERROR(in_msg) VOCPP_TRACE(VOCPP::TraceLevel::ERROR, in_msg)
#define VOCPP_TRACE_DEBUG(in_msg) VOCPP_TRACE(VOCPP::TraceLevel::DEBUG, in_msg)

} //namespace Utils
} //namespace VOCPP

#endif /* VOCPP_TRACING_IMPL_H */
