"""
PC-side Path Estimation
---------------------
This script runs on the PC to:
1. Receive match data from the OpenMV Nicla Vision via UDP
2. Collect data for 8 seconds
3. Compute camera motion and 2D path prediction (Y-Z plane)

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

def form_transf(R, t):
    """Form transformation matrix from rotation and translation."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def compute_essential_matrix(F, K):
    """Compute essential matrix from fundamental matrix and camera calibration."""
    E = K.T @ F @ K
    return E

def decomp_essential_mat(E, q1, q2, K):
    """Decompose essential matrix to get rotation and translation."""
    def sum_z_cal_relative_scale(R, t):
        # Form projection matrices
        P1 = np.hstack((K, np.zeros((3, 1))))  # First camera is at origin
        P2 = np.hstack((K @ R, K @ t.reshape(3, 1)))  # Second camera
        
        # Triangulate points
        hom_Q1 = cv2.triangulatePoints(P1, P2, q1.T, q2.T)
        hom_Q2 = np.matmul(form_transf(R, t), hom_Q1)
        
        # Convert to non-homogeneous coordinates
        uhom_Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
        uhom_Q2 = hom_Q2[:3, :] / hom_Q2[3, :]
        
        # Count points in front of both cameras
        sum_of_pos_z_Q1 = sum(uhom_Q1[2, :] > 0)
        sum_of_pos_z_Q2 = sum(uhom_Q2[2, :] > 0)
        
        # Calculate relative scale
        relative_scale = np.mean(np.linalg.norm(uhom_Q1.T[:-1] - uhom_Q1.T[1:], axis=-1) / 
                               np.linalg.norm(uhom_Q2.T[:-1] - uhom_Q2.T[1:], axis=-1))
        
        return sum_of_pos_z_Q1 + sum_of_pos_z_Q2, relative_scale

    R1, R2, t = cv2.decomposeEssentialMat(E)
    t = np.squeeze(t)
    pairs = [[R1, t], [R1, -t], [R2, t], [R2, -t]]
    z_sums = []
    relative_scales = []
    
    for R, t in pairs:
        z_sum, scale = sum_z_cal_relative_scale(R, t)
        z_sums.append(z_sum)
        relative_scales.append(scale)
    
    right_pair_idx = np.argmax(z_sums)
    right_pair = pairs[right_pair_idx]
    relative_scale = relative_scales[right_pair_idx]
    R1, t = right_pair
    t = t * relative_scale
    
    return R1, t

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
    
    return F, mask, prev_points_undist, curr_points_undist

def plot_path(positions):
    """Plot the estimated camera path in Y-Z plane."""
    positions = np.array(positions)
    plt.figure(figsize=(10, 10))
    plt.plot(positions[:, 1], positions[:, 2], 'b-', label='Camera Path')
    plt.scatter(positions[:, 1], positions[:, 2], c='r', marker='o')
    plt.axis('equal')
    plt.grid(True)
    plt.title('Estimated Camera Path (Y-Z Plane)')
    plt.xlabel('Y (meters)')
    plt.ylabel('Z (meters)')
    plt.legend()
    plt.show()

def main():
    print("Starting path estimation...")
    print("Will collect data for 8 seconds...")
    
    # Initialize UDP socket with larger buffer
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)  # 64KB buffer
    udp_socket.bind(('0.0.0.0', 8080))
    udp_socket.settimeout(0.1)  # 100ms timeout
    
    # Initialize data collection
    start_time = time.time()
    match_data = deque(maxlen=1000)  # Store last 1000 matches
    positions = [(0, 0, 0)]  # Start at origin (x, y, z)
    current_position = np.array([0.0, 0.0, 0.0])
    current_rotation = np.eye(3)
    
    try:
        while time.time() - start_time < 8:
            try:
                data, addr = udp_socket.recvfrom(65536)  # Increased buffer size
                match_data.append(json.loads(data.decode()))
                print(".", end="", flush=True)
            except socket.timeout:
                continue
            except json.JSONDecodeError as e:
                print(f"\nError decoding JSON: {e}")
                continue
            except Exception as e:
                print(f"\nError receiving data: {e}")
                continue
    
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
                F, mask, prev_points_undist, curr_points_undist = compute_fundamental_matrix(prev_points, curr_points)
                
                if F is not None:
                    # Compute essential matrix
                    E = compute_essential_matrix(F, K)
                    
                    # Decompose essential matrix with improved method
                    R, t = decomp_essential_mat(E, prev_points_undist, curr_points_undist, K)
                    
                    # Update position and rotation
                    current_rotation = current_rotation @ R
                    
                    # Update position with scaled translation
                    translation = current_rotation @ t
                    current_position = current_position + translation
                    positions.append(tuple(current_position))
        
        except Exception as e:
            print(f"Error processing frame {i}: {e}")
    
    # Plot the path
    plot_path(positions)

if __name__ == "__main__":
    main() 