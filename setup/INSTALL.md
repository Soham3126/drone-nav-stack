# Installing on a fresh computer

Tested on **Ubuntu 22.04**, NVIDIA GPU (RTX), 8+ CPU cores, 16 GB RAM. Takes about 1-2 hours, most of it downloads and
ArduPilot's build. Follows the guidebook's chapter 3 with the Humble changes listed in `../HUMBLE_NOTES.md`.

## 1. ROS 2 Humble and tools
Follow the official guide: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html> (desktop install),
then:
```
sudo apt install -y ros-dev-tools
sudo rosdep init; rosdep update
```

## 2. Gazebo Harmonic (OSRF repository) and the ROS bridge for it
```
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic ros-humble-ros-gzharmonic
```

## 3. ROS packages for SLAM, planning and tests
```
sudo apt install -y \
  ros-humble-cartographer ros-humble-cartographer-ros \
  ros-humble-nav2-planner ros-humble-nav2-smac-planner ros-humble-nav2-theta-star-planner \
  ros-humble-nav2-costmap-2d ros-humble-nav2-lifecycle-manager ros-humble-nav2-map-server \
  ros-humble-nav2-msgs ros-humble-nav2-util \
  ros-humble-robot-state-publisher ros-humble-xacro ros-humble-tf2-tools \
  ros-humble-rosbag2-storage-mcap ros-humble-foxglove-bridge ros-humble-topic-tools
```

## 4. Python libraries and tools
```
sudo apt install -y python3-numpy python3-pandas python3-matplotlib python3-opencv python3-yaml python3-psutil \
  tmux stress-ng
python3 -m pip install --user pymavlink evo "scipy>=1.15" py-spy
```
(Ubuntu's own scipy 1.8 does not work with numpy 2.x, hence scipy from pip.)

## 5. NVIDIA driver
```
sudo ubuntu-drivers install && sudo reboot
nvidia-smi
```

## 6. ArduPilot SITL, pinned to Copter 4.7.2
```
cd ~ && git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y && . ~/.profile
git fetch --tags && git checkout Copter-4.7.2 && git submodule update --init --recursive
./waf configure --board sitl && ./waf copter
git describe --tags --exact-match --match 'Copter-*'     # must print Copter-4.7.2
```
If ArduPilot's installer added a line to `~/.bashrc` that activates a Python virtual environment, comment it out
(ROS needs the system Python).

## 7. ArduPilot's Gazebo plugin
```
sudo apt install -y libgz-sim8-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-gl
cd ~ && git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo && mkdir -p build && cd build
GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j$(nproc)
```

## 8. This repository
The scripts expect it at `~/drone_ws/src`:
```
mkdir -p ~/drone_ws && git clone https://github.com/<your-account>/<this-repo>.git ~/drone_ws/src
echo "source ~/drone_ws/src/setup/env.sh" >> ~/.bashrc && source ~/.bashrc
ros_env
cd ~/drone_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install
```

## 9. Check
```
cd ~/drone_ws && colcon test --packages-select drone_bridge && colcon test-result --verbose   # 64 tests pass
cd src && scripts/run_test.sh B8 w0_box_room truth 1500                                       # 7 take-offs, PASS
```
