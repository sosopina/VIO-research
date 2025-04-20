"""
Feature Detection and Matching with Arrows for Visual Odometry
------------------------------------------------------------
This script detects keypoints in both current and previous frames,
matches them using descriptors, and draws arrows showing the matches.

Author: sosopina
Date: 2025-04-17
"""

import sensor
import time
import image

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
ARROW_COLOR = (255, 0, 0)           # Red color for arrows
CIRCLE_SIZE = 3                     # Size of circles at keypoints
LINE_THICKNESS = 2                  # Thickness of match lines

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

def draw_match_arrows(img, matches, prev_kpoints, curr_kpoints):
    """
    Draw arrows showing feature matches between frames.
    
    Args:
        img: The image to draw on
        matches: List of match objects
        prev_kpoints: Keypoints from previous frame
        curr_kpoints: Keypoints from current frame
    """
    valid_matches = 0
    for match in matches.match():
        # Get the matched keypoints (they are tuples with x,y coordinates)
        prev_kp = prev_kpoints[match[0]]
        curr_kp = curr_kpoints[match[1]]
        
        # Check if keypoints are within image bounds
        if (0 <= prev_kp[0] < img.width() and 0 <= prev_kp[1] < img.height() and
            0 <= curr_kp[0] < img.width() and 0 <= curr_kp[1] < img.height()):
            
            # Draw line from previous to current keypoint
            img.draw_line(prev_kp[0], prev_kp[1], curr_kp[0], curr_kp[1], 
                         color=ARROW_COLOR, thickness=LINE_THICKNESS)
            
            # Draw circles at keypoint locations
            img.draw_circle(prev_kp[0], prev_kp[1], CIRCLE_SIZE, color=ARROW_COLOR, thickness=2)
            img.draw_circle(curr_kp[0], curr_kp[1], CIRCLE_SIZE, color=ARROW_COLOR, thickness=2)
            
            valid_matches += 1
            
    print("Valid matches drawn:", valid_matches)

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
    current_kpoints = current_frame.find_keypoints(max_keypoints=150, threshold=10, normalized=True)
    
    # Draw current frame keypoints in black
    if current_kpoints:
        draw_keypoints(current_frame, current_kpoints, color=(0, 0, 0))
        print("Current frame keypoints:", len(current_kpoints))
    
    # Draw previous frame keypoints in white
    if last_kpoints:
        draw_keypoints(current_frame, last_kpoints, color=(255, 255, 255))
        print("Previous frame keypoints:", len(last_kpoints))
        
        # Match keypoints between frames
        matches = image.match_descriptor(last_kpoints, current_kpoints, threshold=85)
        
        # Draw match arrows
        if matches:
            print("Total matches found:", len(matches.match()))
            draw_match_arrows(current_frame, matches, last_kpoints, current_kpoints)
    
    # Store current keypoints for next iteration
    last_kpoints = current_kpoints
    
    # Flush the frame buffer
    sensor.flush()
    
    # Print FPS
    print("FPS:", clock.fps()) 
