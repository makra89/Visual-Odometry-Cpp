import Vocpp
import cv2
import numpy as np

'''
The python wrapper is not really functional at the moment since it cannot
handle the conversion of numpy to opencv types yet (mat, point2f, ...). 
For numpy there exists a SWIG addon, but for opencv I have not found anything
useful. Probably the easiest way to get this going is to get rid of opencv types
in the public interface
'''

master = Vocpp.Master()

# Create camera calibration 
focLength = 7.18856e+02
camCenterX = 6.071928000000e+02
camCenterY = 1.852157000000e+02
skew = 0.0
monoCalib = Vocpp.MonoCameraCalibration(focLength, camCenterX, camCenterY, skew)

# Load calibration to master
ret = master.LoadCalibration(monoCalib)


