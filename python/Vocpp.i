
%module Vocpp

%{
#define SWIG_FILE_WITH_INIT
#include<Vocpp_Interface/Frame.h>
#include<Vocpp_Interface/DeltaCameraPose.h>
#include<Vocpp_Interface/CameraPose.h>
#include<Vocpp_Calibration/MonoCameraCalibration.h>
#include<Vocpp_Master/Master.h>
%}

%include "numpy.i"
%init %{
import_array();
%}

%apply (float* IN_FARRAY2, int DIM1, int DIM2) {(float* const in_grayImgData, int in_width, int in_height)};

// Include all parts of Frame.h that do not use opencv types
namespace VOCPP
{
static int s_invalidFrameId;
static bool IsValidFrameId(const int in_frameId);

class Frame
{
public:
    Frame();
    Frame(float* const in_grayImgData, int in_width, int in_height, int in_frameId);
    const int GetId();
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
    MonoCameraCalibration(const float& in_focLength, const float& in_cameraCentX,
        const float& in_cameraCentY, const float& in_skew);
    bool IsValid() const;
};
} // namespace Calibration
} // namespace VOCPP

%include<Vocpp_Interface/DeltaCameraPose.h>
%include<Vocpp_Interface/CameraPose.h>
%include<Vocpp_Master/Master.h>

