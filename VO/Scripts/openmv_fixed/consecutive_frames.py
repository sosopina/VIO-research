"""
Consecutive Frame Display for Visual Odometry
--------------------------------------------
This script captures and displays consecutive frames for visual odometry analysis.
It shows the half of previous frame on top and and half of current frame on bottom.

Author: [sosopina]
Date: [2025-04-17]
"""

import sensor, image, time

# Initialize camera sensor
sensor.reset()                      # Reset and initialize the camera
sensor.set_pixformat(sensor.GRAYSCALE)  # Set pixel format to grayscale
sensor.set_framesize(sensor.QVGA)   # Set resolution to QVGA (320x240)
sensor.skip_frames(time=2000)       # Let the camera adjust to lighting
sensor.set_framebuffers(3)          # Allocate 3 frame buffers for frame comparison

# Initialize clock for FPS measurement
clock = time.clock()

# Get frame dimensions
frame_width = sensor.width()        # Get frame width (320 for QVGA)
frame_height = sensor.height()      # Get frame height (240 for QVGA)
half_height = frame_height // 2     # Calculate half height for splitting

# Allocate extra frame buffer for frame comparison
frame = sensor.alloc_extra_fb(frame_width, frame_height, sensor.GRAYSCALE)

while True:
    # Start FPS measurement
    clock.tick()
    
    # Capture previous frame
    last_frame = sensor.get_fb()
    
    # Add small delay to ensure sufficient motion between frames
    time.sleep_us(500000)  # 500ms delay
    
    # Capture current frame
    current_frame = sensor.snapshot()
    
    # Calculate frame similarity (useful for motion estimation)
    frame_similarity = current_frame.get_similarity(last_frame)
    print("Frame similarity (stdev):", frame_similarity.stdev())
    
    # Create image for displaying previous frame
    displayed_last_frame = image.Image(frame_width, half_height, sensor.GRAYSCALE)
    
    # Copy bottom half of previous frame to the display image
    displayed_last_frame.replace(last_frame, roi=(0, half_height, frame_width, half_height))
    
    # Draw previous frame on top of current frame
    current_frame.draw_image(displayed_last_frame, 0, 0)
    
    # Flush the frame buffer
    sensor.flush()
    
    # Print current FPS
    print("FPS:", clock.fps()) 