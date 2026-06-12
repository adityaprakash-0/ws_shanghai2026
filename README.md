# WorldSkills Shanghai 2026 - Skill 23 (Autonomous Mobile Robotics)

This repository contains a fully isolated and self-contained ROS 2 software stack for the Studica VMX hardware. It uses localized C++ drivers to prevent system-wide dependency conflicts and avoids the need for global installations.

## 🚀 Quick Setup Guide

Follow these steps immediately after cloning to configure your environment and build the workspace on a new system.

### 1. Clone the Repository
Ensure you clone this repository directly into the home directory and name the folder `skill23_ws`:
```bash
cd ~
git clone [https://github.com/adityaprakash-0/ws_shanghai2026.git](https://github.com/adityaprakash-0/ws_shanghai2026.git) skill23_ws
cd skill23_ws

2. Update Hardcoded Paths (If username is not 'vmx')

Important: The CMake configuration currently relies on absolute paths pointing to /home/vmx/. If your Raspberry Pi uses a different username, run this command to dynamically update the paths to match your current system:
Bash

find src/studica_ws/src/studica_control -type f -name "CMakeLists.txt" -exec sed -i "s|/home/vmx/skill23_ws|$HOME/skill23_ws|g" {} +

3. Configure Environment Paths (Permanent)

The compiler and runtime need to know where the localized drivers live. Run this block of commands once to permanently add the paths to your terminal profile:
Bash

echo 'export CPLUS_INCLUDE_PATH=$HOME/skill23_ws/local_sysroot/include/studica_drivers:$CPLUS_INCLUDE_PATH' >> ~/.bashrc
echo 'export LIBRARY_PATH=$HOME/skill23_ws/local_sysroot/lib:$LIBRARY_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$HOME/skill23_ws/local_sysroot/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

4. Link Runtime Library to ROS Workspace

Inject the local library path directly into the ROS 2 setup script so the nodes can find the custom drivers when launched:
Bash

echo 'export LD_LIBRARY_PATH=$HOME/skill23_ws/local_sysroot/lib:$LD_LIBRARY_PATH' >> ~/skill23_ws/install/setup.bash

5. Grant Hardware Permissions

The VMX HAL requires elevated access to physically control the GPIO pins and I2C buses. Run the permission script once:
Bash

./src/studica_ws/scripts/setup_permissions.sh ~/skill23_ws

6. Build the Workspace

With the environment configured, the build system will automatically pull the localized headers and libraries:
Bash

colcon build --packages-select studica_control

7. Launch the Hardware Bridge

Source the new build and start the control server to connect to the VMX:
Bash

source ~/skill23_ws/install/setup.bash
ros2 launch studica_control studica_launch.py
