/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2021 Manuel Kraus
*/

#ifndef VOCPP_TRACING_H
#define VOCPP_TRACING_H

namespace VOCPP
{

enum class TraceLevel
{
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3
};

void SetTraceLevel(const TraceLevel& in_traceLevel);

/**
  * /brief Tracer class
  */
class Tracer
{
public:
    virtual ~Tracer() {}
    virtual void receiveTrace(const TraceLevel& in_traceLevel, const char* const in_msg) = 0;
};

/**
  * /brief Register Tracer interface implementation, use nullptr to deregister a tracer
  */
bool RegisterTracer(Tracer* in_tracerImpl);

} //namespace VOCPP

#endif /* VOCPP_TRACING_H */
