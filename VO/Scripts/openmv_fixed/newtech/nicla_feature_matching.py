"""
Feature Detection and Matching with PC Communication
--------------------------------------------------
This script runs on the OpenMV Nicla Vision to:
1. Detect and match features between frames
2. Send match data to PC for fundamental matrix estimation
3. Display visualization on the Nicla's screen

Author: sosopina
Date: 2024-04-17
"""

import sensor
import time
import image
import json
import socket
import network

# Initialize camera sensor
sensor.reset()
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((320, 240))
sensor.skip_frames(time=2000)
sensor.set_framebuffers(3)

# Initialize network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

# Connect to WiFi (replace with your network details)
SSID = "ZTE_DF5934"
PASSWORD = "oussama2001"
print("Attempting to connect to WiFi...")
wlan.connect(SSID, PASSWORD)

# Wait for connection with timeout
connection_timeout = 10  # seconds
start_time = time.time()
while not wlan.isconnected():
    if time.time() - start_time > connection_timeout:
        print("WiFi connection timeout!")
        break
    time.sleep(0.1)
    print("Connecting to WiFi...")

if wlan.isconnected():
    print("Connected to WiFi")
    print("IP address:", wlan.ifconfig()[0])
else:
    print("Failed to connect to WiFi!")
    raise Exception("WiFi connection failed")

# Initialize UDP socket
try:
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    PC_IP = "192.168.0.235"  # Replace with your PC's IP address
    PC_PORT = 8080
    print(f"UDP socket initialized. Will send to {PC_IP}:{PC_PORT}")
except Exception as e:
    print("Error initializing UDP socket:", e)
    raise

# Constants
KEYPOINTS_SIZE = 8
ARROW_COLOR = (255, 0, 0)
CIRCLE_SIZE = 3
LINE_THICKNESS = 2

def draw_keypoints(img, keypoints, color):
    """Draw detected keypoints on the image with specified color."""
    if keypoints:
        img.draw_keypoints(keypoints, size=KEYPOINTS_SIZE, color=color)

def draw_match_arrows(img, matches, prev_kpoints, curr_kpoints):
    """Draw arrows showing feature matches between frames."""
    valid_matches = 0
    match_data = []  # List to store match data for PC
    
    for match in matches.match():
        prev_kp = prev_kpoints[match[0]]
        curr_kp = curr_kpoints[match[1]]
        
        if (0 <= prev_kp[0] < img.width() and 0 <= prev_kp[1] < img.height() and
            0 <= curr_kp[0] < img.width() and 0 <= curr_kp[1] < img.height()):
            
            # Draw visualization
            img.draw_line(prev_kp[0], prev_kp[1], curr_kp[0], curr_kp[1], 
                         color=ARROW_COLOR, thickness=LINE_THICKNESS)
            img.draw_circle(prev_kp[0], prev_kp[1], CIRCLE_SIZE, color=ARROW_COLOR, thickness=2)
            img.draw_circle(curr_kp[0], curr_kp[1], CIRCLE_SIZE, color=ARROW_COLOR, thickness=2)
            
            # Store match data for PC
            match_data.append({
                'prev_x': float(prev_kp[0]),
                'prev_y': float(prev_kp[1]),
                'curr_x': float(curr_kp[0]),
                'curr_y': float(curr_kp[1])
            })
            
            valid_matches += 1
    
    # Send match data to PC if we have matches
    if match_data:
        try:
            data = json.dumps(match_data) + '\n'
            print(f"Sending {len(match_data)} matches to PC...")
            bytes_sent = udp_socket.sendto(data.encode(), (PC_IP, PC_PORT))
            print(f"Sent {bytes_sent} bytes")
        except Exception as e:
            print("Error sending data:", e)
            print("Data that failed to send:", data[:100])  # Print first 100 chars of data
    
    print("Valid matches:", valid_matches)

# Initialize variables
last_kpoints = None
current_kpoints = None
clock = time.clock()

print("Starting main loop...")
while True:
    try:
        clock.tick()
        
        # Capture current frame
        current_frame = sensor.snapshot()
        
        # Detect keypoints in current frame
        current_kpoints = current_frame.find_keypoints(max_keypoints=150, threshold=10, normalized=True)
        
        # Draw current frame keypoints
        if current_kpoints:
            draw_keypoints(current_frame, current_kpoints, color=(0, 0, 0))
            print(f"Found {len(current_kpoints)} keypoints in current frame")
        
        # Process matches if we have previous keypoints
        if last_kpoints:
            draw_keypoints(current_frame, last_kpoints, color=(255, 255, 255))
            matches = image.match_descriptor(last_kpoints, current_kpoints, threshold=85)
            
            if matches:
                print(f"Found {len(matches.match())} matches")
                draw_match_arrows(current_frame, matches, last_kpoints, current_kpoints)
        
        # Store current keypoints for next iteration
        last_kpoints = current_kpoints
        
        # Flush the frame buffer
        sensor.flush()
        
        # Print FPS
        print("FPS:", clock.fps())
        
    except Exception as e:
        print("Error in main loop:", e)
        time.sleep(1)  # Wait a bit before continuing 