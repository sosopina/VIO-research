"""
Feature Detection in current and next frame for Visual Odometry
-------------------------------------------------
This script detects keypoints in both current and previous frames.
Current frame keypoints are drawn in black, previous frame keypoints in white.
while displaying in the preview the current frame only 

Author: sosopina
Date: 2025-04-17
"""

import sensor
import time

# Initialize camera sensor
sensor.reset()
sensor.set_contrast(3)              # Increase contrast for better feature detection
sensor.set_gainceiling(16)          # Set maximum gain
sensor.set_pixformat(sensor.GRAYSCALE)  # Set pixel format to grayscale
sensor.set_framesize(sensor.QVGA)   # Set resolution to QVGA (320x240)
sensor.set_windowing((320, 240))    # Set window size
sensor.skip_frames(time=2000)       # Let the camera adjust to lighting
sensor.set_framebuffers(3)          # Allocate frame buffers for frame comparison

# Constants
KEYPOINTS_SIZE = 8                  # Size of keypoint markers in pixels

def draw_keypoints(img, keypoints, color):
    """
    Draw detected keypoints on the image with specified color.
    
    Args:
        img: The image to draw on
        keypoints: List of KeyPoint objects
        color: RGB color tuple for the keypoints
    """
    if keypoints:
        img.draw_keypoints(keypoints, size=KEYPOINTS_SIZE, color=color)

# Initialize variables
last_kpoints = None                 # Store keypoints from previous frame
current_kpoints = None              # Store keypoints from current frame
clock = time.clock()                # For FPS measurement

while True:
    # Start FPS measurement
    clock.tick()
    
    # Capture current frame
    current_frame = sensor.snapshot()
    
    # Detect keypoints in current frame
    current_kpoints = current_frame.find_keypoints(max_keypoints=100, threshold=20, scale_factor=1.2)
    
    # Draw current frame keypoints in black
    if current_kpoints:
        draw_keypoints(current_frame, current_kpoints, color=(0, 0, 0))
        print("Current frame keypoints:", len(current_kpoints))
    
    # Draw previous frame keypoints in white
    if last_kpoints:
        draw_keypoints(current_frame, last_kpoints, color=(255, 255, 255))
        print("Previous frame keypoints:", len(last_kpoints))
    
    # Store current keypoints for next iteration
    last_kpoints = current_kpoints
    
    # Flush the frame buffer
    sensor.flush()
    
    # Print FPS
    print("FPS:", clock.fps()) 