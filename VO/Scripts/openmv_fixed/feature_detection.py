"""
Feature Detection for Visual Odometry
------------------------------------
This script detects and displays keypoints in the camera feed using the FAST corner detector.
These keypoints are essential for feature-based visual odometry.

According to OpenMV documentation:
- find_keypoints() uses the FAST corner detector
- Keypoints are returned as KeyPoint objects with x, y, score, and angle attributes
- The threshold parameter controls the sensitivity of the detector

Author: sosopina
Date: 2025-04-17
"""

import sensor
import time

# Initialize camera sensor
sensor.reset()
sensor.set_contrast(3)              # Increase contrast for better feature detection
sensor.set_gainceiling(16)          # Set maximum gain
sensor.set_framesize(sensor.QVGA)   # Set resolution to QVGA (320x240)
sensor.set_windowing((320, 240))    # Set window size
sensor.set_pixformat(sensor.GRAYSCALE)  # Set pixel format to grayscale
sensor.skip_frames(time=2000)       # Let the camera adjust to lighting

# Constants
KEYPOINTS_SIZE = 8                  # Size of keypoint markers in pixels

def draw_keypoints(img, keypoints):
    """
    Draw detected keypoints on the image and print their coordinates.
    According to OpenMV docs, keypoints are KeyPoint objects with:
    - x, y: coordinates
    - score: corner strength
    - angle: orientation (if available)
    
    Args:
        img: The image to draw on
        keypoints: List of KeyPoint objects
    """
    if keypoints:
        print("Number of keypoints detected:", len(keypoints))
        img.draw_keypoints(keypoints, size=KEYPOINTS_SIZE)

# Initialize variables
kpoints = None                      # Store detected keypoints
clock = time.clock()                # For FPS measurement

while True:
    # Start FPS measurement
    clock.tick()
    
    # Capture image
    img = sensor.snapshot()
    
    # Detect keypoints using FAST corner detector
    # Parameters from OpenMV docs:
    # - max_keypoints: Maximum number of keypoints to detect (default: 100)
    # - threshold: Threshold for corner detection (default: 20, lower = more keypoints)
    # - scale_factor: Scale factor for multi-scale detection (default: 1.2)
    # - normalized: Whether to normalize keypoint coordinates (default: False)
    kpoints = img.find_keypoints(max_keypoints=100, threshold=20, scale_factor=1.2, normalized=False)
    
    # Draw and print keypoints if any are found
    if kpoints:
        draw_keypoints(img, kpoints)
    
    # Print FPS
    print("FPS:", clock.fps()) 