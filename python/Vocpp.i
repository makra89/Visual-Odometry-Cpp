
%module Vocpp

%{
#define SWIG_FILE_WITH_INIT
#include<opencv2/core/core.hpp>
#include<Vocpp_Interface/Frame.h>
#include<Vocpp_Interface/DeltaPose.h>
#include<Vocpp_Calibration/MonoCameraCalibration.h>
#include<Vocpp_Master/Master.h>
%}

%include<Vocpp_Interface/Frame.h>
%include<Vocpp_Interface/DeltaPose.h>
%include<Vocpp_Calibration/MonoCameraCalibration.h>
%include<Vocpp_Master/Master.h>

