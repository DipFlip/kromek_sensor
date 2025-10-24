# Kromek Sigma50 ROS2 HIDAPI Driver

ROS2 Humble driver for Kromek Sigma50 gamma-ray detector using HIDAPI.

**This is the ROS2 port** of the original ROS1 kernel module driver. It uses HIDAPI instead of kernel modules, making it compatible with:
- ✅ Secure Boot (no module signing needed)
- ✅ Docker containers (no privileged kernel access)
- ✅ Standard Linux systems (just USB access required)

## Original ROS1 Version

The original ROS1 version using kernel modules is available on the `main` branch.
This ROS2 HIDAPI version is on the `ros2-hidapi-humble` branch.

## Quick Start

### Docker (Recommended)

Build the Docker image:
```bash
docker build -t kromek_ros2 -f docker/Dockerfile .
```

Run the container:
```bash
docker run --rm --privileged \
  -v /dev:/dev \
  -v /dev/bus/usb:/dev/bus/usb \
  kromek_ros2 \
  ros2 launch kromek_ros2_hidapi kromek_hidapi.launch.py
```

### Native Install

Install dependencies:
```bash
sudo apt-get install -y \
  ros-humble-rclcpp \
  ros-humble-std-msgs \
  libhidapi-dev \
  libhidapi-libusb0
```

Build:
```bash
cd ~/ros2_ws/src
git clone -b ros2-hidapi-humble git@github.com:DipFlip/kromek_sensor.git
cd ~/ros2_ws
colcon build --packages-select kromek_ros2_hidapi
```

Run:
```bash
ros2 launch kromek_ros2_hidapi kromek_hidapi.launch.py
```

## Features

- **Configurable detector settings**: gain, LLD (energy threshold)
- **Flexible histogram binning**: 1-4096 bins
- **Multiple operating modes**: spectroscopy, real-time, survey
- **No kernel modules required**: uses HIDAPI userspace library

## Configuration

See [detailed documentation](README.md) for all parameters.

### Example Configurations

**High-resolution spectroscopy:**
```bash
ros2 launch kromek_ros2_hidapi kromek_hidapi.launch.py \
  integration_seconds:=10 num_bins:=4096 lld:=100
```

**Real-time monitoring:**
```bash
ros2 launch kromek_ros2_hidapi kromek_hidapi.launch.py \
  num_bins:=20 publish_rate_hz:=10.0
```

**Survey/mapping:**
```bash
ros2 launch kromek_ros2_hidapi kromek_hidapi.launch.py \
  integration_seconds:=2 num_bins:=50 gain:=120
```

## Topics

- `/kromek/sum` - Total gamma-ray counts
- `/kromek/raw` - Energy histogram
- `/kromek/info` - Detector serial numbers  
- `/kromek/status` - Device settings (optional)

## Attribution

Based on:
- Original DipFlip/kromek_sensor ROS1 driver (kernel module approach)
- LBL-ANP HIDAPI implementation
- Ported to ROS2 Humble by SLAM Dunk team

## License

See LICENSE file.
