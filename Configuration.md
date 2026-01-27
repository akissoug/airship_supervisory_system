# Configuration Guide

This guide explains all configuration parameters for the ArduPilot Supervisory System.

## Configuration File

The main configuration file is `config/supervisory_params.yaml`. This file is loaded by the launch file and can be overridden at runtime.

## Full Configuration Reference

```yaml
# ArduPilot Supervisory System Configuration
# For fixed-wing aircraft (including WIG craft)

power_monitor:
  ros__parameters:
    # === Battery Thresholds ===
    low_battery_warning: 30.0      # Warning threshold (%)
    critical_battery: 20.0         # RTL trigger threshold (%)
    min_voltage_per_cell: 3.5      # Minimum safe voltage per cell (V)
    battery_cells: 4               # Number of cells (3S=3, 4S=4, 6S=6)
    
    # === Current Monitoring ===
    cruise_current_estimate: 15.0  # Estimated cruise current (A)
    current_averaging_window: 10   # Samples for averaging
    
    # === Fixed-Wing Performance ===
    cruise_speed: 18.0             # Cruise airspeed (m/s)
    safety_margin: 0.3             # Safety margin for return (30%)
    
    # === Timing ===
    check_interval: 2.0            # Battery check interval (s)
    startup_grace_period: 30.0     # Grace period at startup (s)
    
    # === Modes ===
    sitl_mode: true                # true=SITL, false=real hardware
    enable_rtl_trigger: true       # Enable automatic RTL on low battery

fault_detector:
  ros__parameters:
    # === GPS Thresholds ===
    min_satellites: 8              # Minimum satellites for healthy GPS
    min_hdop: 2.5                  # Maximum acceptable HDOP
    gps_timeout: 5.0               # GPS data timeout (s)
    
    # === IMU Thresholds ===
    max_vibration: 30.0            # Maximum vibration (m/s²)
    imu_timeout: 2.0               # IMU data timeout (s)
    
    # === Airspeed Thresholds (Fixed-Wing) ===
    min_airspeed: 12.0             # Stall warning speed (m/s)
    max_airspeed: 35.0             # Overspeed warning (m/s)
    airspeed_timeout: 5.0          # Airspeed data timeout (s)
    
    # === Fault Detection ===
    check_interval: 1.0            # Health check interval (s)
    consecutive_failures: 3        # Failures before fault declared
    startup_grace_period: 30.0     # Grace period at startup (s)
    
    sitl_mode: true

mission_supervisor:
  ros__parameters:
    # === Mission Timeouts ===
    mission_timeout: 3600.0        # Max mission duration (s) - 1 hour
    waypoint_timeout: 300.0        # Max time per waypoint (s) - 5 min
    
    # === Stuck Detection ===
    stuck_detection_radius: 50.0   # Movement threshold (m)
    stuck_detection_time: 60.0     # Time to detect stuck (s)
    enable_stuck_detection: true   # Enable/disable stuck detection
    
    # === Safety ===
    min_altitude_for_actions: 20.0 # Min altitude for actions (m AGL)
    
    # === Timing ===
    check_interval: 3.0            # Supervision interval (s)
    startup_grace_period: 30.0     # Grace at startup (s)
    mission_start_grace: 45.0      # Grace after mission start (s)
    
    sitl_mode: true

energy_manager:
  ros__parameters:
    # === Aircraft Performance ===
    cruise_speed: 18.0             # Cruise airspeed (m/s)
    climb_rate: 3.0                # Climb rate (m/s)
    descent_rate: 2.0              # Descent rate (m/s)
    loiter_radius: 80.0            # Fixed-wing loiter radius (m)
    
    # === Energy Reserves ===
    reserve_percentage: 20.0       # Battery reserve for landing (%)
    headwind_margin: 5.0           # Assumed headwind (m/s)
    
    # === Timing ===
    calculation_interval: 5.0      # Calculation rate (s)
    
    sitl_mode: true

telemetry_handler:
  ros__parameters:
    # === Logging ===
    log_to_file: true              # Enable file logging
    log_directory: "/home/user/flight_logs"  # Log location
    
    # === Data Collection ===
    telemetry_rate: 1.0            # Publishing rate (Hz)
    position_history_size: 1000    # Position history buffer
    battery_history_size: 500      # Battery history buffer
    
    sitl_mode: true

emergency_manager:
  ros__parameters:
    # === Emergency Response ===
    enable_auto_emergency: true    # Enable automatic responses
    emergency_loiter_altitude: 100.0  # Emergency loiter altitude (m)
    emergency_loiter_radius: 150.0    # Emergency loiter radius (m)
    
    # === Timing ===
    emergency_timeout: 300.0       # Max emergency duration (s)
    max_emergency_retries: 3       # Command retry attempts
    command_cooldown: 15.0         # Time between commands (s)
    
    sitl_mode: true
```

## Configuration Profiles
(according to leterature research)
### SITL Testing Profile
```yaml
# For simulation testing real flight scenarios
power_monitor:
  ros__parameters:
    sitl_mode: true
    critical_battery: 15.0         # Lower for longer testing
    enable_rtl_trigger: true

fault_detector:
  ros__parameters:
    sitl_mode: true
    min_satellites: 6              
    consecutive_failures: 5        # More tolerant

mission_supervisor:
  ros__parameters:
    sitl_mode: true
    mission_timeout: 7200.0        # 2 hours for testing
    waypoint_timeout: 600.0        # 10 min per waypoint
```

### Real Hardware Profile
```yaml
# For real flight - strict thresholds
power_monitor:
  ros__parameters:
    sitl_mode: false
    critical_battery: 25.0         # Higher for safety
    enable_rtl_trigger: true

fault_detector:
  ros__parameters:
    sitl_mode: false
    min_satellites: 8
    consecutive_failures: 3

mission_supervisor:
  ros__parameters:
    sitl_mode: false
    mission_timeout: 1800.0        # 30 min max
    enable_stuck_detection: true
```

### Small prototype case
```yaml
power_monitor:
  ros__parameters:
    battery_cells: 3               # 3S battery
    cruise_current_estimate: 8.0   # Lower current
    cruise_speed: 12.0             # Slower aircraft

fault_detector:
  ros__parameters:
    min_airspeed: 8.0              # Lower stall speed
    max_airspeed: 25.0             # Lower max speed

energy_manager:
  ros__parameters:
    cruise_speed: 12.0
    loiter_radius: 50.0            # Tighter turns
```

### Large prorotype example
```yaml
power_monitor:
  ros__parameters:
    battery_cells: 6               # 6S battery
    cruise_current_estimate: 25.0  # Higher current
    cruise_speed: 22.0             # Faster aircraft

fault_detector:
  ros__parameters:
    min_airspeed: 15.0
    max_airspeed: 45.0

mission_supervisor:
  ros__parameters:
    mission_timeout: 7200.0        # 2 hour missions
    waypoint_timeout: 600.0        # Photo waypoints

energy_manager:
  ros__parameters:
    cruise_speed: 22.0
    loiter_radius: 120.0           # Larger turns
```

### WIG Craft
```yaml
power_monitor:
  ros__parameters:
    battery_cells: 4
    cruise_current_estimate: 12.0  # Lower in ground effect
    cruise_speed: 15.0
    safety_margin: 0.4             # Higher for water ops

fault_detector:
  ros__parameters:
    min_airspeed: 10.0             # Lower for ground effect
    max_airspeed: 25.0

energy_manager:
  ros__parameters:
    cruise_speed: 15.0
    headwind_margin: 8.0           # Sea conditions
```

## Runtime Parameter Override

Override parameters at launch:
```bash
ros2 launch ardupilot_supervisory supervisory_system.launch.py \
  sitl_mode:=false
```

Override for individual nodes:
```bash
ros2 run ardupilot_supervisory power_monitor --ros-args \
  -p critical_battery:=25.0 \
  -p battery_cells:=6
```

## Parameter Verification

Check current parameters:
```bash
ros2 param list /power_monitor
ros2 param get /power_monitor critical_battery
```

Set parameter at runtime:
```bash
ros2 param set /power_monitor critical_battery 25.0
```

## Important Notes

1. **sitl_mode** - Always set to `false` for real hardware
2. **Battery cells** - Must match the actual battery configuration
3. **Airspeed limits** - Calibrate for each case
4. **Grace periods** - Allow time for sensors to initialize
5. **Timeouts** - Balance between safety and false positives
