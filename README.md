# ArduPilot ROS2 Supervisory System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-Plane%204.x-green)](https://ardupilot.org/)

A comprehensive ROS2-based supervisory system for ArduPilot fixed-wing aircraft, including WIG (Wing-in-Ground effect) craft. This system provides real-time monitoring, fault detection, energy management, and emergency handling capabilities.


## Features

- **Battery & Power Monitoring** - Real-time battery status, voltage per cell monitoring, automatic RTL on low battery
- **Fault Detection** - GPS, IMU, magnetometer, and airspeed sensor health monitoring
- **Mission Supervision** - Waypoint tracking, mission timeout detection, stuck detection
- **Energy Management** - Safe return range calculations, flight time estimation
- **Telemetry Aggregation** - Centralized data logging and status publishing
- **Emergency Coordination** - Automated failsafe responses and emergency handling

## Table of Contents

- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Nodes Documentation](#nodes-documentation)
- [Topics Reference](#topics-reference)
- [Testing](#testing)
- [Hardware Deployment](#hardware-deployment)
- [Contributing](#contributing)
- [License](#license)

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        SUPERVISORY SYSTEM                               │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │   Power     │  │   Fault     │  │  Mission    │  │   Energy    │     │
│  │  Monitor    │  │  Detector   │  │ Supervisor  │  │  Manager    │     │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘     │
│         │                │                │                │            │
│  ┌──────┴────────────────┴────────────────┴────────────────┴──────┐     │
│  │                    Telemetry Handler                           │     │
│  └─────────────────────────────┬──────────────────────────────────┘     │
│                                │                                        │
│  ┌─────────────────────────────┴───────────────────────────────────┐    │
│  │                    Emergency Manager                            │    │
│  └─────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                           MAVROS2                                       │
│  Subscribed: /mavros/state, /mavros/battery, etc...                     │
│  Services: /mavros/set_mode, /mavros/cmd/arming                         │
└─────────────────────────────────────────────────────────────────────────┘
                                 │
                    ┌────────────┴────────────┐
                    ▼                         ▼
           ┌───────────────┐         ┌───────────────┐
           │  ArduPilot    │         │  ArduPilot    │
           │    SITL       │         │   Hardware    │
           │ (Simulation)  │         │  (Pixhawk)    │
           └───────────────┘         └───────────────┘
```

## Prerequisites

### Software Requirements

| Software | Version | Notes |
|----------|---------|-------|
| Ubuntu | 22.04 LTS | Tested on this version |
| ROS2 | Humble | Required |
| MAVROS2 | 2.x | Communication bridge |
| ArduPilot | 4.x | SITL or hardware |
| Python | 3.10+ | Included with Ubuntu 22.04 |

### Hardware Requirements (for real deployment)

- Pixhawk-compatible flight controller (in our case Pixhahk 6X)
- USB cable for bench testing
- Telemetry radio (for field operations)
- RC transmitter with manual override capability

## Installation

### Step 1: Install ROS2 Humble

```bash
# Add ROS2 repository
sudo apt update && sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions -y

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install MAVROS2

```bash
# Install MAVROS2
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y

# Install GeographicLib datasets
cd /opt/ros/humble/lib/mavros
sudo ./install_geographiclib_datasets.sh
```

### Step 3: Install ArduPilot SITL (for simulation)

```bash
# Clone ArduPilot
mkdir -p ~/ardupilot_ws && cd ~/ardupilot_ws
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Install prerequisites
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build ArduPlane SITL
./waf configure --board sitl
./waf plane
```

### Step 4: Clone and Build This Package

```bash
# Create ROS2 workspace
mkdir -p ~/ardupilot_ws/ros2_ws/src
cd ~/ardupilot_ws/ros2_ws/src

# Clone this repository
git clone https://github.com/YOUR_USERNAME/ardupilot_ros2_supervisory.git

# Build
cd ~/ardupilot_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ardupilot_supervisory
source install/setup.bash
```

## Configuration

### Main Configuration File

Edit `config/supervisory_params.yaml` to customize thresholds:

```yaml
power_monitor:
  ros__parameters:
    low_battery_warning: 30.0    # Warning at 30%
    critical_battery: 20.0       # RTL at 20%
    battery_cells: 4             # 4S battery
    sitl_mode: true              # Set false for real hardware
```

See Configuration Guide for detailed parameter descriptions.

## Usage

### With SITL Simulation

**Terminal 1: Start ArduPilot SITL**
```bash
cd ~/ardupilot_ws/ardupilot/ArduPlane
sim_vehicle.py -v ArduPlane --console --map
```

**Terminal 2: Start MAVROS2**
```bash
source /opt/ros/humble/setup.bash

# For SITL only
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:="udp://0.0.0.0:14550@" \
  -p target_system_id:=1 \
  -p target_component_id:=1

# With Mission Planner on another PC (e.g., 192.168.0.102)
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:="udp://0.0.0.0:14550@" \
  -p gcs_url:="udp://@192.168.0.102:14550" \
  -p target_system_id:=1 \
  -p target_component_id:=1
```

**Terminal 3: Launch Supervisory System**
```bash
cd ~/ardupilot_ws/ros2_ws
source install/setup.bash

# Launch all nodes
ros2 launch ardupilot_supervisory supervisory_system.launch.py sitl_mode:=true

# Or run individual nodes
ros2 run ardupilot_supervisory power_monitor
```

### With Real Hardware

See Hardware Deployment Guide

## Nodes Documentation

| Node | Description |
|------|-------------|
| `power_monitor` | Battery monitoring and RTL trigger | 
| `fault_detector` | Sensor health monitoring |
| `mission_supervisor` | Mission execution monitoring |
| `energy_manager` | Safe return calculations |
| `telemetry_handler` | Data aggregation and logging |
| `emergency_manager` | Emergency coordination |

## Topics Reference

### Published Topics (Supervisory System)

| Topic | Type | Description |
|-------|------|-------------|
| `/supervisory/power_status` | `std_msgs/String` | JSON with battery status, flight time, distances |
| `/supervisory/power_alert` | `std_msgs/String` | RTL trigger alerts |
| `/supervisory/fault_status` | `std_msgs/String` | Sensor health status |
| `/supervisory/active_faults` | `std_msgs/String` | List of active faults |
| `/supervisory/mission_status` | `std_msgs/String` | Mission execution status |
| `/supervisory/current_waypoint` | `std_msgs/String` | Waypoint progress events |
| `/supervisory/energy_status` | `std_msgs/String` | Range and time calculations |
| `/supervisory/safe_return_available` | `std_msgs/Bool` | Safe return feasibility |
| `/supervisory/telemetry` | `std_msgs/String` | Aggregated telemetry data |
| `/supervisory/emergency_status` | `std_msgs/String` | Emergency manager status |
| `/supervisory/emergency_alert` | `std_msgs/String` | Emergency event alerts |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | ROS2 standard diagnostics |

### Subscribed Topics (from MAVROS2)

| Topic | Type | Used By |
|-------|------|---------|
| `/mavros/state` | `mavros_msgs/State` | All nodes |
| `/mavros/battery` | `sensor_msgs/BatteryState` | power_monitor, energy_manager, telemetry |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | power_monitor, mission_supervisor, energy_manager |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | mission_supervisor, telemetry |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | All nodes (airspeed, altitude) |
| `/mavros/imu/data` | `sensor_msgs/Imu` | fault_detector, telemetry |
| `/mavros/imu/mag` | `sensor_msgs/MagneticField` | fault_detector |
| `/mavros/mission/waypoints` | `mavros_msgs/WaypointList` | mission_supervisor |
| `/mavros/mission/reached` | `mavros_msgs/WaypointReached` | mission_supervisor |
| `/mavros/home_position/home` | `mavros_msgs/HomePosition` | mission_supervisor, energy_manager |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | fault_detector |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | fault_detector |

See Topics Reference for detailed message formats.

## Testing

### SITL Testing Scenarios

```bash
# Run all tests
cd ~/ardupilot_ws/ros2_ws
source install/setup.bash
python3 src/ardupilot_supervisory/tests/run_tests.py

# Individual scenario tests
python3 src/ardupilot_supervisory/tests/test_low_battery.py
python3 src/ardupilot_supervisory/tests/test_gps_loss.py
python3 src/ardupilot_supervisory/tests/test_mission_execution.py
```

See Testing Guide for detailed test procedures and video recordings.

## Hardware Deployment

### Connection Options

| Method | Use Case | Configuration |
|--------|----------|---------------|
| USB/Serial | Bench testing | `fcu_url:="serial:///dev/ttyACM0:115200"` |
| Telemetry Radio | Field operations | `fcu_url:="serial:///dev/ttyUSB0:57600"` |

### Safety Checklist

- [ ] Remove propellers for bench testing
- [ ] Set `sitl_mode: false` in configuration
- [ ] Verify RC transmitter override works
- [ ] Have manual override ready at all times

See Hardware Deployment Guide


## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [ArduPilot](https://ardupilot.org/) - Open source autopilot
- [MAVROS](https://github.com/mavlink/mavros) - ROS/ROS2 MAVLink interface
- [ROS2](https://ros.org/) - Robot Operating System
