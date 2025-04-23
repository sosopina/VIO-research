"""
PC-side Path Estimation
---------------------
This script runs on the PC to:
1. Receive match data from the OpenMV Nicla Vision via UDP
2. Collect data for 15 seconds
3. Compute camera motion and 2D path prediction

Author: sosopina
Date: 2024-04-17
"""

import socket
import json
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
from collections import deque

# Camera calibration matrix from calibration results
K = np.array([
    [433.928287, 0.000000, 144.118474],
    [0.000000, 432.834569, 116.390615],
    [0.000000, 0.000000, 1.000000]
])

# Distortion coefficients
dist_coeffs = np.array([0.148701, 1.345802, -0.000248, 0.000795, -12.083145])

def compute_essential_matrix(F, K):
    """Compute essential matrix from fundamental matrix and camera calibration."""
    E = K.T @ F @ K
    return E

def decompose_essential_matrix(E):
    """Decompose essential matrix to get rotation and translation."""
    U, _, Vt = np.linalg.svd(E)
    W = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    
    # Possible rotations
    R1 = U @ W @ Vt
    R2 = U @ W.T @ Vt
    
    # Possible translations
    t = U[:, 2]
    
    # Ensure proper rotation matrices
    if np.linalg.det(R1) < 0:
        R1 = -R1
    if np.linalg.det(R2) < 0:
        R2 = -R2
    
    return R1, R2, t

def compute_fundamental_matrix(prev_points, curr_points):
    """Compute fundamental matrix using the 8-point algorithm."""
    # Ensure points are in the correct format
    prev_points = np.array(prev_points, dtype=np.float32)
    curr_points = np.array(curr_points, dtype=np.float32)
    
    # Undistort points using calibration
    prev_points_undist = cv2.undistortPoints(prev_points.reshape(-1, 1, 2), K, dist_coeffs)
    curr_points_undist = cv2.undistortPoints(curr_points.reshape(-1, 1, 2), K, dist_coeffs)
    
    # Reshape back to Nx2
    prev_points_undist = prev_points_undist.reshape(-1, 2)
    curr_points_undist = curr_points_undist.reshape(-1, 2)
    
    # Compute fundamental matrix
    F, mask = cv2.findFundamentalMat(
        prev_points_undist, 
        curr_points_undist, 
        cv2.FM_8POINT
    )
    
    return F, mask

def plot_path(positions):
    """Plot the estimated camera path."""
    positions = np.array(positions)
    plt.figure(figsize=(10, 10))
    plt.plot(positions[:, 0], positions[:, 1], 'b-', label='Camera Path')
    plt.scatter(positions[:, 0], positions[:, 1], c='r', marker='o')
    plt.axis('equal')
    plt.grid(True)
    plt.title('Estimated Camera Path')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend()
    plt.show()

def main():
    print("Starting path estimation...")
    print("Will collect data for 15 seconds...")
    
    # Initialize UDP socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('0.0.0.0', 8080))
    udp_socket.settimeout(0.1)  # 100ms timeout
    
    # Initialize data collection
    start_time = time.time()
    match_data = deque(maxlen=1000)  # Store last 1000 matches
    positions = [(0, 0)]  # Start at origin
    current_position = np.array([0.0, 0.0])
    current_rotation = np.eye(3)
    
    try:
        while time.time() - start_time < 8:
            try:
                data, addr = udp_socket.recvfrom(4096)
                match_data.append(json.loads(data.decode()))
                print(".", end="", flush=True)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"\nError receiving data: {e}")
    
    except KeyboardInterrupt:
        print("\nStopping data collection...")
    finally:
        udp_socket.close()
    
    print(f"\nCollected {len(match_data)} frames")
    
    # Process collected data
    for i in range(len(match_data) - 1):
        try:
            # Get consecutive frames
            prev_matches = match_data[i]
            curr_matches = match_data[i + 1]
            
            # Extract points
            prev_points = []
            curr_points = []
            
            # Process matches to ensure corresponding points
            for match in prev_matches:
                prev_points.append([match['prev_x'], match['prev_y']])
                curr_points.append([match['curr_x'], match['curr_y']])
            
            if len(prev_points) >= 8:
                # Convert to numpy arrays
                prev_points = np.array(prev_points, dtype=np.float32)
                curr_points = np.array(curr_points, dtype=np.float32)
                
                # Compute fundamental matrix
                F, mask = compute_fundamental_matrix(prev_points, curr_points)
                
                if F is not None:
                    # Compute essential matrix
                    E = compute_essential_matrix(F, K)
                    
                    # Decompose essential matrix
                    R1, R2, t = decompose_essential_matrix(E)
                    
                    # Choose the correct rotation (simplified)
                    R = R1  # In practice, you'd need to check which is correct
                    
                    # Update position and rotation
                    current_rotation = current_rotation @ R
                    current_position = current_position + current_rotation[:2, 2] * 0.1  # Scale factor
                    positions.append(tuple(current_position))
        
        except Exception as e:
            print(f"Error processing frame {i}: {e}")
    
    # Plot the path
    plot_path(positions)

if __name__ == "__main__":
    main() 