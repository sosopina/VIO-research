# Visual Odometry and Visual-Inertial Odometry Research

This repository contains implementations of Visual Odometry (VO) and Visual-Inertial Odometry (VIO) across different platforms:

## Project Structure

### 1. Visual Odometry (VO) on PC
- Implemented using OpenCV and Python
- Focus on feature-based visual odometry
- Real-time camera motion estimation
- Includes:
  - Feature detection and matching
  - Motion estimation
  - Trajectory visualization

### 2. Visual Odometry on Nicla Vision
- Implemented using MicroPython
- Optimized for low-power embedded device
- Real-time processing on constrained hardware
- Features:
  - Efficient feature detection
  - Frame-to-frame matching
  - Motion estimation
  - Visual feedback

### 3. Future Work: Visual-Inertial Odometry (VIO)
- Planned implementation combining:
  - Visual features from camera
  - Inertial measurements from IMU
  - Sensor fusion for improved accuracy
  - Implementation on both PC and Nicla Vision

## Repository Structure

```
VIO-research/
├── VO/                      # Visual Odometry implementations
│   ├── Scripts/             # Python and MicroPython scripts
│   │   ├── openmv/          # Original OpenMV scripts for Nicla Vision
│   │   └── openmv_fixed/    # Clean, documented OpenMV scripts
│   └── Dataset/             # Dataset for testing and validation
└── README.md                # Project documentation
```

## Current Focus

1. **Visual Odometry Implementation**:
   - Feature detection and matching
   - Frame-to-frame motion estimation
   - Real-time performance optimization

2. **Platform-Specific Optimizations**:
   - PC: Full-featured implementation
   - Nicla Vision: Resource-constrained optimization

## Future Goals

1. **Visual-Inertial Odometry (VIO)**:
   - Integration of IMU data
   - Sensor fusion algorithms
   - Improved accuracy and robustness

2. **Cross-Platform Consistency**:
   - Unified algorithm implementation
   - Platform-specific optimizations
   - Performance benchmarking

## Development Status

- [x] Basic VO implementation on PC
- [x] VO implementation on Nicla Vision
- [ ] VIO implementation (planned)
- [ ] Cross-platform optimization
- [ ] Performance benchmarking
