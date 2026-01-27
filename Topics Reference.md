# Topics Reference

This document provides detailed information about all ROS2 topics used by the ArduPilot Supervisory System.

## Table of Contents

- [Supervisory System Topics (Published)](#supervisory-system-topics-published)
- [MAVROS2 Topics (Subscribed)](#mavros2-topics-subscribed)
- [Message Formats](#message-formats)

---

## Supervisory System Topics (Published)

These topics are published by the supervisory nodes and can be subscribed to by other systems (e.g., ground station, logging, visualization).

### `/supervisory/power_status`

**Type:** `std_msgs/String` (JSON)

**Publisher:** `power_monitor`

**Rate:** 1 Hz

**Description:** Real-time power system status including battery level, voltage, current, and flight time estimates.

**JSON Format:**
```json
{
  "timestamp": 1704067200.123,
  "battery_percentage": 75.5,
  "voltage": 16.2,
  "current": 12.5,
  "armed": true,
  "mode": "AUTO",
  "flight_time": 245.5,
  "distance_to_home": 1250.3,
  "remaining_flight_time": 1800.0,
  "return_time_needed": 95.0,
  "rtl_triggered": false,
  "mission_count": 1
}
```

**Fields:**
| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | float | Unix timestamp |
| `battery_percentage` | float | Battery level (0-100%) |
| `voltage` | float | Total battery voltage (V) |
| `current` | float | Average current draw (A) |
| `armed` | bool | Vehicle armed state |
| `mode` | string | Current flight mode |
| `flight_time` | float | Current flight duration (seconds) |
| `distance_to_home` | float | Distance to home position (meters) |
| `remaining_flight_time` | float | Estimated remaining flight time (seconds) |
| `return_time_needed` | float | Time needed to return home (seconds) |
| `rtl_triggered` | bool | Whether RTL has been triggered |
| `mission_count` | int | Number of missions in session |

---

### `/supervisory/power_alert`

**Type:** `std_msgs/String` (JSON)

**Publisher:** `power_monitor`

**Rate:** Event-driven (only when alert occurs)

**Description:** Alerts when power-related events trigger actions like RTL.

**JSON Format:**
```json
{
  "type": "RTL_TRIGGERED",
  "reason": "Critical battery: 19.5%",
  "timestamp": 1704067200.123
}
```

**Alert Types:**
| Type | Description |
|------|-------------|
| `RTL_TRIGGERED` | RTL mode requested due to power issue |
| `LOW_BATTERY_WARNING` | Battery below warning threshold |
| `LOW_VOLTAGE_WARNING` | Voltage per cell below minimum |

---

### `/supervisory/fault_status`

**Type:** `std_msgs/String` (JSON)

**Publisher:** `fault_detector`

**Rate:** 1 Hz

**Description:** Overall sensor health status.

**JSON Format:**
```json
{
  "timestamp": 1704067200.123,
  "sensors": {
    "gps": true,
    "imu": true,
    "mag": true,
    "airspeed": true,
    "rc": true
  },
  "active_faults": [],
  "armed": true,
  "mode": "AUTO"
}
```

**Sensor Health Fields:**
| Sensor | Description | Healthy When |
|--------|-------------|--------------|
| `gps` | GPS receiver | Fix acquired, sufficient satellites |
| `imu` | Inertial measurement unit | Data received, vibration within limits |
| `mag` | Magnetometer | Valid field strength readings |
| `airspeed` | Airspeed sensor | Within min/max thresholds |
| `rc` | RC receiver | Signal received, sufficient quality |

---

### `/supervisory/active_faults`

**Type:** `std_msgs/String` (JSON)

**Publisher:** `fault_detector`

**Rate:** Event-driven (only when faults detected)

**Description:** List of currently active faults.

**JSON Format:**
```json
{
  "faults": ["GPS_TIMEOUT", "AIRSPEED_LOW"],
  "timestamp": 1704067200.123
}
```

**Fault Codes:**
| Fault Code | Description |
|------------|-------------|
| `GPS_TIMEOUT` | No GPS data received within timeout |
| `GPS_UNHEALTHY` | GPS fix lost or insufficient satellites |
| `IMU_TIMEOUT` | No IMU data received |
| `IMU_UNHEALTHY` | Vibration exceeds threshold |
| `MAG_TIMEOUT` | No magnetometer data |
| `MAG_UNHEALTHY` | Invalid magnetic field readings |
| `AIRSPEED_LOW` | Below stall speed threshold |
| `AIRSPEED_HIGH` | Above overspeed threshold |
| `RC_TIMEOUT` | No RC signal |
| `RC_UNHEALTHY` | Poor RC signal quality |

---

### `/supervisory/mission_status`

**Type:** `std_msgs/String` (JSON)

**Publisher:** `mission_supervisor`

**Rate:** 1 Hz

**Description:** Mission execution status and progress.

**JSON Format:**
```json
{
  "timestamp": 1704067200.123,
  "state": "MISSION",
  "mode": "AUTO",
  "armed": true,
  "mission_active": true,
  "current_waypoint": 3,
  "total_waypoints": 10,
  "mission_time": 180.5,
  "altitude": 85.2,
  "stuck_detected": false,
  "mission_count": 1,
  "statistics": {
    "total_missions": 1,
    "successful_missions": 0,
    "failed_missions": 0,
    "waypoints_reached": 3,
    "stuck_events": 0
  }
}
```

**Mission States:**
| State | Description |
|-------|-------------|
| `IDLE` | Not in mission mode |
| `ARMED` | Armed, waiting for mission |
| `TAKEOFF` | Takeoff in progress |
| `MISSION` | Executing waypoint mission |
| `LOITER` | Loitering at position |
| `RTL` | Returning to launch |
| `LANDING` | Landing in progress |
| `DISARMED` | Vehicle disarmed |

---

### `/supervisory/current_waypoint`

**Type:** `std_msgs/String` (JSON)

**Publisher:** `mission_supervisor`

**Rate:** Event-driven (on waypoint reached)

**Description:** Waypoint progress events.

**JSON Format:**
```json
{
  "event": "waypoint_reached",
  "waypoint": 3,
  "total": 10,
  "timestamp": 1704067200.123
}
```

---

### `/supervisory/energy_status`

**Type:** `std_msgs/String` (JSON)

**Publisher:** `energy_manager`

**Rate:** 0.2 Hz (every 5 seconds)

**Description:** Energy calculations and range estimates.

**JSON Format:**
```json
{
  "timestamp": 1704067200.123,
  "battery_percentage": 75.5,
  "usable_percentage": 55.5,
  "max_range_m": 15000.0,
  "safe_return_range_m": 7500.0,
  "distance_to_home_m": 1250.3,
  "estimated_return_time_s": 95.0,
  "safe_to_continue": true,
  "groundspeed": 18.5
}
```

**Fields:**
| Field | Description |
|-------|-------------|
| `usable_percentage` | Battery minus reserve (for calculations) |
| `max_range_m` | Maximum theoretical range |
| `safe_return_range_m` | Range while ensuring safe return |
| `safe_to_continue` | Whether it's safe to continue mission |

---

### `/supervisory/safe_return_available`

**Type:** `std_msgs/Bool`

**Publisher:** `energy_manager`

**Rate:** 0.2 Hz (every 5 seconds)

**Description:** Simple boolean indicating if safe return is possible.

**Value:**
- `true` - Current distance is within safe return range
- `false` - May not have enough energy to return safely

---

### `/supervisory/telemetry`

**Type:** `std_msgs/String` (JSON)

**Publisher:** `telemetry_handler`

**Rate:** 1 Hz (configurable)

**Description:** Aggregated telemetry from all sensors.

**JSON Format:**
```json
{
  "timestamp": 1704067200.123,
  "battery": {
    "percentage": 75.5,
    "voltage": 16.2
  },
  "state": {
    "armed": true,
    "mode": "AUTO"
  },
  "position": {
    "lat": 37.7749,
    "lon": -122.4194,
    "alt": 85.2
  },
  "vfr": {
    "airspeed": 18.5,
    "groundspeed": 20.1,
    "heading": 270
  },
  "statistics": {
    "start_time": 1704067000.0,
    "max_altitude": 95.3,
    "max_speed": 22.5,
    "min_battery": 75.5
  },
  "history_points": 180
}
```

---

### `/supervisory/emergency_status`

**Type:** `std_msgs/String` (JSON)

**Publisher:** `emergency_manager`

**Rate:** 1 Hz

**Description:** Emergency manager status.

**JSON Format:**
```json
{
  "timestamp": 1704067200.123,
  "emergency_active": false,
  "emergency_type": "NONE",
  "armed": true,
  "mode": "AUTO",
  "retries": 0,
  "auto_enabled": true
}
```

**Emergency Types:**
| Type | Description |
|------|-------------|
| `NONE` | No emergency |
| `LOW_BATTERY` | Battery below threshold |
| `CRITICAL_BATTERY` | Battery critical |
| `SENSOR_FAILURE` | Sensor health issue |
| `GPS_LOSS` | GPS signal lost |
| `COMMUNICATION_LOSS` | Link lost |
| `GEOFENCE_BREACH` | Outside geofence |
| `MANUAL_TRIGGER` | Manually triggered |

---

### `/supervisory/emergency_alert`

**Type:** `std_msgs/String` (JSON)

**Publisher:** `emergency_manager`

**Rate:** Event-driven

**Description:** Emergency event alerts.

**JSON Format:**
```json
{
  "type": "EMERGENCY",
  "emergency_type": "LOW_BATTERY",
  "reason": "Battery at 19%",
  "timestamp": 1704067200.123
}
```

---

### `/diagnostics`

**Type:** `diagnostic_msgs/DiagnosticArray`

**Publisher:** `fault_detector`

**Rate:** 1 Hz

**Description:** Standard ROS2 diagnostics for sensor health.

This follows the ROS2 diagnostics standard format and can be viewed with:
```bash
ros2 topic echo /diagnostics
```

---

## MAVROS2 Topics (Subscribed)

These topics are published by MAVROS2 and subscribed to by the supervisory nodes.

> **Important:** All MAVROS2 subscriptions use `BEST_EFFORT` QoS reliability to match MAVROS2's publishing policy.

### Topic Summary

| Topic | Type | QoS | Subscribers |
|-------|------|-----|-------------|
| `/mavros/state` | `mavros_msgs/State` | BEST_EFFORT | All nodes |
| `/mavros/battery` | `sensor_msgs/BatteryState` | BEST_EFFORT | power_monitor, energy_manager, telemetry |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | BEST_EFFORT | power_monitor, mission_supervisor, energy_manager |
| `/mavros/global_position/raw/fix` | `sensor_msgs/NavSatFix` | BEST_EFFORT | fault_detector |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | BEST_EFFORT | mission_supervisor, telemetry |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | BEST_EFFORT | telemetry |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | BEST_EFFORT | All nodes |
| `/mavros/imu/data` | `sensor_msgs/Imu` | BEST_EFFORT | fault_detector, telemetry |
| `/mavros/imu/mag` | `sensor_msgs/MagneticField` | BEST_EFFORT | fault_detector |
| `/mavros/mission/waypoints` | `mavros_msgs/WaypointList` | BEST_EFFORT | mission_supervisor |
| `/mavros/mission/reached` | `mavros_msgs/WaypointReached` | BEST_EFFORT | mission_supervisor |
| `/mavros/home_position/home` | `mavros_msgs/HomePosition` | BEST_EFFORT | mission_supervisor, energy_manager |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | BEST_EFFORT | fault_detector |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | BEST_EFFORT | fault_detector |

---

## Viewing Topics

### List All Supervisory Topics
```bash
ros2 topic list | grep supervisory
```

### Monitor a Topic
```bash
# Single message
ros2 topic echo /supervisory/power_status --once

# Continuous monitoring
ros2 topic echo /supervisory/power_status

# With JSON formatting
ros2 topic echo /supervisory/power_status | python3 -m json.tool
```

### Check Topic Info
```bash
ros2 topic info /supervisory/power_status -v
```

### Record Topics
```bash
# Record all supervisory topics
ros2 bag record /supervisory/power_status /supervisory/fault_status /supervisory/mission_status -o flight_log
```
