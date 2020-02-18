import Vocpp
import cv2
import numpy as np
import sys
import os
import matplotlib.pyplot as plt
import getopt
import configparser
from matplotlib.animation import FuncAnimation

# Define range for which the position is drawn
plot_x_range = 200
plot_y_range = 200

def usage():
    print("\npython runKittiDemo.py -i [<kitti_seq0_dir>] -ref [<ref_file>]")
    print("\nRun comparison of position estimation from Vocpp to ground truth ")
    print("for KITTI sequence 0.")
 

if __name__ == "__main__":
    
    ground_truth_file = None
    seq0_image_dir = None

     # Parse command line
    try:
        opts, args = getopt.getopt(sys.argv[1:], "i:r:")
    except getopt.GetoptError:
        sys.exit(-1)

    for opt, arg in opts:
        if opt == "-i":
            seq0_image_dir = arg
        elif opt == "-r":
            ground_truth_file  = arg

    if seq0_image_dir is None or ground_truth_file is None:
        usage()
        sys.exit(-1)

    # Read ground truth poses
    ground_truth_positions = []
    with open(ground_truth_file, 'r') as f:
                lines = f.readlines()
                for line in lines:
                    T_w_cam0 = np.fromstring(line, dtype=float, sep=' ')
                    T_w_cam0 = T_w_cam0.reshape(3, 4)
                    T_w_cam0 = np.vstack((T_w_cam0, [0, 0, 0, 1]))
                    ground_truth_positions.append(T_w_cam0[0:3, 3])
    # Ground truth poses given with respect to camera system of first frame
    # Therefore, WCS and ground truth coordinate system are rotated with respect to each other               
    ground_truth_positions = np.array(ground_truth_positions)

    master = Vocpp.Master()

    # Create camera calibration 
    foc_length = 7.18856e+02
    cam_center_x = 6.071928000000e+02
    cam_center_y = 1.852157000000e+02
    skew = 0.0
    mono_calib = Vocpp.MonoCameraCalibration(foc_length, cam_center_x, cam_center_y, skew)

    # Load calibration to master
    ret = master.LoadCalibration(mono_calib)

    #positions_wcs = np.empty((0,3), dtype=float)

    # Setup plot
    fig = plt.figure("Vocpp vs. Ground Truth")
    ax = plt.axes(xlim=(-plot_x_range, plot_x_range), ylim=(-plot_y_range, plot_y_range))
    positions_wcs_plot, = ax.plot([], [], lw=2)
    ground_truth_plot, = ax.plot([], [], lw=2)

    inputFiles = []
    for file in os.listdir(seq0_image_dir):
        if file.endswith(".png"):
            inputFiles.append(seq0_image_dir + file)
        
    positions_wcs_x = []
    positions_wcs_y = [] 
    frame_id = 0

    def init():
        """ Initialize plots """
        positions_wcs_plot.set_data([], [])
        ground_truth_plot.set_data([], [])
        plt.xlabel("X axis WCS [arbitrary]")
        plt.ylabel("Y axis WCS [arbitrary]")

        return positions_wcs_plot, ground_truth_plot

    def update(frame):
        """ Update plots for new frame + new ground truth """

        # Get frame ID from global scope
        global frame_id

        img = cv2.imread(frame, cv2.IMREAD_GRAYSCALE)
        # For some reason it can happen that empty images are read
        if(img.data is not None):
            img = np.float32(img) * 1/255.0
            frame = Vocpp.Frame(np.transpose(img), frame_id)
                
            # Feed image to master and get last absolute pose in WCS
            master.FeedNextFrame(frame)
            pose = master.GetLastPose() 
                
            # Update plot
            positions_wcs_x.append(pose.GetCamCenterTranslation().x)
            positions_wcs_y.append(pose.GetCamCenterTranslation().y)
            positions_wcs_plot.set_data(positions_wcs_x, positions_wcs_y)
                
            # Update ground truth plot, axes from camera system have to be rotated
            # Z axis is aligned with X axis of WCS
            ground_truth_plot.set_data(ground_truth_positions[:frame_id,2], -ground_truth_positions[:frame_id,0])

            ax.set_xlim(pose.GetCamCenterTranslation().x - plot_x_range, pose.GetCamCenterTranslation().x + plot_x_range)
            ax.set_ylim(pose.GetCamCenterTranslation().y - plot_y_range, pose.GetCamCenterTranslation().y + plot_y_range)
        frame_id += 1

        return positions_wcs_plot, ground_truth_plot

    # Run animation
    ani = FuncAnimation(fig, update, frames=inputFiles, init_func=init, blit=True, interval=1)

    plt.show()
