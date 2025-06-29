# OCR-VLM-Based Vision-Guided Navigation

This repository contains a ROS 2 Humble workspace implementing a vision-guided navigation system that integrates Optical Character Recognition (OCR), Vision-Language Models (VLM), and robot control capabilities. The system enables a robot to perceive text in its environment, interpret it using language models, and respond with appropriate navigation actions.
 
### ⏯️ 🎥 *Click the image above to watch the video*

[![Video Preview](https://github.com/user-attachments/assets/d6bd426b-96cb-418f-8b62-942a9190d80d)](https://youtu.be/vHWTwBpvzg8?si=6s6c5gHJRV15vQ6i)



---
## **Simulation in action and VLM Output**

<table>
  <tr>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/5c981b94-42e0-4624-a9df-1b97dfe78c42" alt="Screenshot from 2025-05-14 20-34-21" width="600">
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/7d7d356c-7d73-4e79-9164-bce325c322ab" alt="Figure_1" width="400">
    </td>
  </tr>
</table>

![Screenshot from 2025-06-29 15-09-21](https://github.com/user-attachments/assets/64a779b1-eb22-4936-b13c-5f9b50024945)

## Architecture Overview

The system consists of three main packages that work together to create an end-to-end pipeline:




### 1. `ocr_pkg`
A ROS 2 package that provides OCR capabilities using EasyOCR.

- **Service**: `/ocr_infer`
- **Input**: Camera image
- **Output**: Detected text
- **Functionality**:
  - Text detection and recognition from robot camera images
  - High-accuracy text extraction with confidence scoring

### ⬆️ *Enhancements:*

YOLO11n: Object detection engine for identifying sign boards in the images and feeding the croped portion of the sign boards. 
SORT: Simple Online and Realtime Tracking algorithm for tracking detected sign boards across different frames.

*(This mitigates the issue with OCR reading incorrect (distant) signs and being applied to only the sign board instead the whole picture)*


### 2. `vlm_pkg`
Integrates the MobileVLM Vision-Language Model with the robotic system.

- **Input**: Camera images from `/front_stereo_camera/left/image_raw`
- **Process Flow**:
  1. Receives camera images
  2. Calls OCR service to extract text
  3. Processes image + text with MobileVLM
  4. Generates navigation commands
  5. Sends commands to controller via action server

### 3. `con_pkg`
Provides an action server for robot control and motion execution.

- **Action Server**: `/nav_command`
- **Output Topic**: `/cmd_vel` (Twist messages)
- **Supported Commands**:
  - Forward/go/straight: Moves robot forward
  - Left/turn left: Rotates robot counter-clockwise
  - Right/turn right: Rotates robot clockwise
  - Stop/halt: Stops robot movement

## System Requirements

### Hardware Requirements
- Robot with ROS 2 support (tested with Carter in Isaac Sim)
- Camera mounted on the robot
- Computer with sufficient processing power for ML workloads
- CUDA-compatible GPU (recommended for VLM inference)

### Software Requirements
- ROS 2 Humble
- Python 3.8+
- EasyOCR
- MobileVLM
- CUDA Toolkit (for GPU acceleration)

## Installation

### 1. Install ROS 2 Humble
Follow the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

### 2. Install MobileVLM
MobileVLM is a lightweight Vision-Language Model designed for mobile and edge devices.

1. Clone the MobileVLM repository:
   ```bash
   git clone https://github.com/Meituan-AutoML/MobileVLM.git
   cd MobileVLM
   ```

2. Create and activate a conda environment:
   ```bash
   conda create -n mobilevlm python=3.10 -y
   conda activate mobilevlm
   pip install --upgrade pip
   ```

3. Install required packages:
   ```bash
   pip install -r requirements.txt
   ```

4. If you're using the system Python instead of conda:
   ```bash
   cd ~/VGN
   git clone https://github.com/Meituan-AutoML/MobileVLM.git
   cd MobileVLM
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

### 3. Install Isaac Sim
Isaac Sim is NVIDIA's robotics simulation platform. Our system uses Isaac Sim with the Carter robot.

1. Download and install Isaac Sim following [NVIDIA's instructions](https://developer.nvidia.com/isaac-sim)

2. Set up the simulation environment structure:
   ```bash
   mkdir -p ~/VGN/sim/{assets,Environments,screenshots}
   ```

3. Organize simulation assets:
   ```
   ~/VGN/sim/
   ├── assets/
   │   ├── 3D/
   │   │   └── textures/
   │   ├── images/
   │   │   ├── signs/        # Contains stop signs, direction signs, etc.
   │   │   └── texts/        # Contains text images for OCR testing
   │   │       └── texts/    
   │   ├── isaacsim_defaults/
   │   └── sky/
   ├── Environments/         # Contains simulation world files
   └── screenshots/          # For saving simulation screenshots
   ```

4. Download required assets:
   - Carter robot model comes with Isaac Sim
   - Text and sign assets should be placed in the appropriate directories

### 4. Install ROS 2 Dependencies
```bash
# ROS 2 dependencies
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs python3-opencv

# Python dependencies
pip install easyocr opencv-python matplotlib
```

### 5. Clone and Build the Workspace
```bash
mkdir -p ~/VGN/vgn_ros_ws/src
cd ~/VGN/vgn_ros_ws/src
# Clone this repository or copy the packages

# Build the workspace
cd ~/VGN/vgn_ros_ws
colcon build
```

## Step-by-Step Execution Guide

### Method 1: Using Individual Commands

#### 1. Terminal Setup
Open 3 separate terminal windows and in each one:
```bash
cd ~/VGN/vgn_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### 2. Start Isaac Sim with Carter Robot
1. Launch Isaac Sim:
   ```bash
   cd ~/isaac-sim  # Adjust path to your Isaac Sim installation
   ./python.sh
   ```

2. Load the Carter robot environment:
   - Open the Stage window
   - Select "Environments" and load your Carter robot world
   - Ensure the robot has a camera mounted and configured to publish to `/front_stereo_camera/left/image_raw`

3. Verify the simulation is correctly set up:
   - The robot should be visible in the environment
   - Text signs should be visible in the robot's path
   - Camera should be functioning (you can check using RViz)

#### 3. Start OCR Service Node (Terminal 1)
```bash
ros2 run ocr_pkg ocr.py
```

#### 4. Start Controller Node (Terminal 2)
```bash
ros2 run con_pkg con.py
```

#### 5. Start VLM Orchestrator Node (Terminal 3)
```bash
ros2 run vlm_pkg vlm.py
```

### Method 2: Using Launch File

```bash
# Open a terminal
cd ~/VGN/vgn_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch all nodes at once
ros2 launch con_pkg vgn.launch.py
```

## System Flow

1. **Image Acquisition**: The VLM node subscribes to `/front_stereo_camera/left/image_raw`
2. **OCR Processing**: The image is sent to the OCR service for text extraction
3. **Text Detection**: OCR service detects and extracts text from the image
4. **VLM Analysis**: The image and detected text are analyzed by MobileVLM
5. **Command Generation**: VLM generates appropriate navigation command
6. **Action Execution**: The command is sent to the control node
7. **Robot Motion**: The control node translates the command to robot motion

## Monitoring and Debugging

### Check Node Status
```bash
ros2 node list
```

### Monitor Topics
```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor image topic
ros2 topic echo /front_stereo_camera/left/image_raw/info
```

### Check Action Server
```bash
ros2 action list
ros2 action info /nav_command
```

### Common Issues and Solutions

#### OCR Service Not Available
- **Solution**: Ensure the OCR node is running and check for Python dependency issues
```bash
ros2 service list | grep ocr_infer
```

#### VLM Errors
- **Solution**: Check MobileVLM installation and Python path
```bash
# Verify MobileVLM is in your PATH
python3 -c "import sys; print('\n'.join(sys.path))"
```

#### Navigation Commands Not Working
- **Solution**: Verify action server is running and the communication flow
```bash
# Test action server manually
ros2 action send_goal /nav_command con_pkg/action/NavCommand "{command: 'go forward'}"
```

## Customization

### Modifying OCR Parameters
Edit the OCR node to adjust language settings or confidence thresholds:
```python
# in ocr_pkg/src/ocr.py
self.reader = easyocr.Reader(['en', 'additional_language'], gpu=True)
```

### Changing VLM Model
Edit the VLM node to use a different model:
```python
# in vlm_pkg/src/vlm.py
self.model_path = "different/model/path"  # Change to another model
```

### Adjusting Robot Movement
Edit the controller node to adjust speed or duration:
```python
# in con_pkg/src/con.py
self.linear_speed = 0.7  # Increase forward speed
self.turn_duration = 1.5  # Decrease turning time
```

## Advanced Configuration

### MobileVLM Configuration
You can configure MobileVLM for different performance levels:

1. For higher speed on less powerful hardware:
   ```python
   # in vlm_pkg/src/vlm.py
   args = type('Args', (), {
       # ...existing code...
       "load_8bit": True,  # Enable 8-bit quantization
       "max_new_tokens": 256,  # Reduce token generation
       # ...existing code...
   })()
   ```

2. For higher quality on powerful GPUs:
   ```python
   # in vlm_pkg/src/vlm.py
   args = type('Args', (), {
       # ...existing code...
       "num_beams": 3,  # Enable beam search
       "temperature": 0.2,  # Add some variability
       # ...existing code...
   })()
   ```

### Isaac Sim Camera Configuration
For optimal OCR performance, configure the robot camera in Isaac Sim:

1. Select the camera in the Stage window
2. Set the following properties:
   - Resolution: At least 640x480
   - Field of View: ~60 degrees
   - Position: Mounted where it can see signs at the robot's eye level

### Environment Setup for Testing
Place text signs in the simulation environment:

1. Use the provided textures in `~/VGN/sim/assets/images/signs/` and `~/VGN/sim/assets/images/texts/`
2. Place signs at key locations to test navigation commands:
   - "TURN LEFT" at intersections
   - "GO FORWARD" in hallways
   - "STOP" at endpoints
   - "TURN RIGHT" at corners

## License
MIT

