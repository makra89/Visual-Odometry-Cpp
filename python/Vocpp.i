
%module(directors="1") Vocpp

%{
#define SWIG_FILE_WITH_INIT
#include<Vocpp_Interface/Frame.h>
#include<Vocpp_Interface/Tracing.h>
#include<Vocpp_Interface/DeltaCameraPose.h>
#include<Vocpp_Interface/CameraPose.h>
#include<Vocpp_Calibration/MonoCameraCalibration.h>
#include<Vocpp_Master/Master.h>
%}

%include "numpy.i"
%init %{
import_array();
%}

%apply (double* IN_FARRAY2, int DIM1, int DIM2) {(double* const in_grayImgData, int in_width, int in_height)};

// Include all parts of Frame.h that do not use opencv types
namespace VOCPP
{
static uint32_t s_invalidFrameId;
static bool IsValidFrameId(const uint32_t in_frameId);

class Frame
{
public:
    Frame();
    Frame(double* const in_grayImgData, int in_width, int in_height, int in_frameId);
    const uint32_t GetId();
    const bool IsValid();
};
} // namespace VOCPP

namespace VOCPP
{
namespace Calibration
{
// Also for MonoCameraCalibration we want to hide some interfaces
class MonoCameraCalibration
{
public:
    MonoCameraCalibration();
    MonoCameraCalibration(const double& in_focLength, const double& in_cameraCentX,
        const double& in_cameraCentY, const double& in_skew);
    bool IsValid() const;
};
} // namespace Calibration
} // namespace VOCPP

// Activate polymorphism for this class
%feature("director") VOCPP::Tracer;
%include<Vocpp_Interface/Tracing.h>
%include<Vocpp_Interface/DeltaCameraPose.h>
%include<Vocpp_Interface/CameraPose.h>
%include<Vocpp_Master/Master.h>

