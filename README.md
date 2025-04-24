# OCR-VLM-Based-Vision-Guided-Navigation

This repository contains a ROS 2 Humble workspace (`vgn_ros_ws`) with multiple modular packages. It is designed for integrating computer vision, language models, and autonomous control features in robotic systems.

---

## Packages

### 1. `ocr_pkg`
A ROS 2 Python package that subscribes to camera input and performs Optical Character Recognition (OCR) using [EasyOCR](https://github.com/JaidedAI/EasyOCR).

- **Input Topic**: `/front_stereo_camera/left/image_raw`
- **Functionality**:
  - Subscribes to camera frames
  - Processes one frame every 10 seconds
  - Performs OCR using GPU
  - Displays result with bounding boxes and recognized text

### 2. `vlm_pkg`
A placeholder package for integrating Vision-Language Models (VLMs) with the robotic system.

### 3. `ac_pkg`
A placeholder package intended for autonomous control and decision-making modules.

---

## Folder Structure

```bash
vgn_ros_ws/
├── src/
│   ├── ocr_pkg/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src/
│   │       └── ocr.py
│   ├── vlm_pkg/
│   └── ac_pkg/
```

---

##  Build Instructions

```bash
# Navigate to the workspace root
mkdir vgn_ros_ws/src

# Source your ROS 2 setup
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

---

## Run the OCR Node

```bash
ros2 run ocr_pkg ocr.py
```

Make sure your camera publisher is active and publishing to `/front_stereo_camera/left/image_raw`.

---

## Requirements

```bash
pip install easyocr matplotlib opencv-python
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs
```

---

