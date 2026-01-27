#!/usr/bin/env python3
"""
POWER MONITOR NODE - ArduPilot MAVROS2 Version
Monitors battery status and triggers RTL when battery is low.
for ArduPilot/MAVROS2.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState, NavSatFix
from mavros_msgs.msg import State, VfrHud
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String
from threading import Lock
from collections import deque
import time
import math
import json


class PowerMonitor(Node):
    def __init__(self):
        super().__init__('power_monitor')
        
        # Declare parameters
        self.declare_parameter('low_battery_warning', 30.0)
        self.declare_parameter('critical_battery', 20.0)
        self.declare_parameter('min_voltage_per_cell', 3.5)
        self.declare_parameter('battery_cells', 4)
        self.declare_parameter('cruise_current_estimate', 15.0)
        self.declare_parameter('current_averaging_window', 10)
        self.declare_parameter('cruise_speed', 18.0)
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('startup_grace_period', 30.0)
        self.declare_parameter('sitl_mode', True)
        self.declare_parameter('enable_rtl_trigger', True)
        
        # Get parameters
        self.low_battery = self.get_parameter('low_battery_warning').value
        self.critical_battery = self.get_parameter('critical_battery').value
        self.min_cell_voltage = self.get_parameter('min_voltage_per_cell').value
        self.battery_cells = self.get_parameter('battery_cells').value
        self.cruise_current = self.get_parameter('cruise_current_estimate').value
        self.current_window = self.get_parameter('current_averaging_window').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.check_interval = self.get_parameter('check_interval').value
        self.startup_grace = self.get_parameter('startup_grace_period').value
        self.sitl_mode = self.get_parameter('sitl_mode').value
        self.enable_rtl = self.get_parameter('enable_rtl_trigger').value
        
        # State variables
        self.battery_state = None
        self.vehicle_state = None
        self.global_position = None
        self.home_position = None
        self.vfr_hud = None
        
        self.rtl_triggered = False
        self.lock = Lock()
        self.node_start_time = time.time()
        self.armed_time = None
        self.last_log_time = 0
        
        # Current history for averaging
        self.current_history = deque(maxlen=self.current_window)
        
        # Mission tracking
        self.mission_count = 0
        self.total_flight_time = 0
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/supervisory/power_status', 10)
        self.alert_pub = self.create_publisher(String, '/supervisory/power_alert', 10)
        
        # QoS profile for MAVROS compatibility
        mavros_qos_rel_vol = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE
        )

        mavros_qos_rel_tra = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        mavros_qos_best = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE
        )


        
        # Subscribers - MAVROS2 topics
        self.create_subscription(
            BatteryState, '/mavros/battery',
            self.battery_callback, mavros_qos_best)
        
        self.create_subscription(
            State, '/mavros/state',
            self.state_callback, mavros_qos_rel_tra)
        
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.position_callback, mavros_qos_best)
        
        self.create_subscription(
            VfrHud, '/mavros/vfr_hud',
            self.vfr_callback, mavros_qos_rel_vol)
        
        # Service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        
        # Timer for checks
        self.create_timer(self.check_interval, self.check_battery)
        self.create_timer(1.0, self.publish_status)
        
        self.print_startup_info()
    
    def print_startup_info(self):
        """Print startup configuration"""
        mode_str = "SITL" if self.sitl_mode else "HARDWARE"
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'POWER MONITOR - MAVROS2 VERSION ({mode_str})')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Battery Thresholds:')
        self.get_logger().info(f'  Warning: {self.low_battery}%')
        self.get_logger().info(f'  Critical (RTL): {self.critical_battery}%')
        self.get_logger().info(f'  Min cell voltage: {self.min_cell_voltage}V')
        self.get_logger().info(f'  Battery cells: {self.battery_cells}S')
        self.get_logger().info(f'Fixed-Wing Settings:')
        self.get_logger().info(f'  Cruise speed: {self.cruise_speed} m/s')
        self.get_logger().info(f'  Safety margin: {self.safety_margin * 100}%')
        self.get_logger().info(f'  RTL trigger enabled: {self.enable_rtl}')
        self.get_logger().info('=' * 60)
    
    def battery_callback(self, msg: BatteryState):
        """Process battery state from MAVROS"""
        with self.lock:
            self.battery_state = msg
            
            # Track current for averaging
            if msg.current > 0:
                self.current_history.append(msg.current)
            elif self.sitl_mode:
                # as SITL may not report current properly
                self.current_history.append(self.cruise_current)
    
    def state_callback(self, msg: State):
        """Process vehicle state from MAVROS"""
        with self.lock:
            prev_armed = self.vehicle_state.armed if self.vehicle_state else False
            self.vehicle_state = msg
            
            # Track arming state changes
            if msg.armed and not prev_armed:
                self.handle_arm()
            elif not msg.armed and prev_armed:
                self.handle_disarm()
    
    def position_callback(self, msg: NavSatFix):
        """Process GPS position"""
        with self.lock:
            self.global_position = msg
            
            # Set home position on first valid GPS after arming
            if (self.home_position is None and 
                self.is_armed() and 
                msg.latitude != 0 and msg.longitude != 0):
                self.home_position = (msg.latitude, msg.longitude, msg.altitude)
                self.get_logger().info(
                    f'Home position set: lat={msg.latitude:.6f}, '
                    f'lon={msg.longitude:.6f}, alt={msg.altitude:.1f}m'
                )
    
    def vfr_callback(self, msg: VfrHud):
        """Process VFR HUD data (airspeed, groundspeed, etc.)"""
        with self.lock:
            self.vfr_hud = msg
    
    def is_armed(self):
        """Check if vehicle is armed"""
        return self.vehicle_state is not None and self.vehicle_state.armed
    
    def get_current_mode(self):
        """Get current flight mode"""
        if self.vehicle_state is None:
            return "UNKNOWN"
        return self.vehicle_state.mode
    
    def handle_arm(self):
        """Handle arming event"""
        self.armed_time = time.time()
        self.mission_count += 1
        self.rtl_triggered = False
        self.get_logger().info(f'ARMED - Mission #{self.mission_count}')
    
    def handle_disarm(self):
        """Handle disarming event"""
        if self.armed_time:
            flight_time = time.time() - self.armed_time
            self.total_flight_time += flight_time
            self.get_logger().info(
                f'DISARMED - Flight time: {flight_time:.1f}s, '
                f'Total: {self.total_flight_time:.1f}s'
            )
        self.armed_time = None
    
    def get_flight_time(self):
        """Get current flight time"""
        if self.armed_time is None:
            return 0
        return time.time() - self.armed_time
    
    def is_in_grace_period(self):
        """Check if in startup grace period"""
        elapsed = time.time() - self.node_start_time
        if elapsed < self.startup_grace:
            return True
        if self.armed_time and (time.time() - self.armed_time) < 30:
            return True
        return False
    
    def get_battery_percentage(self):
        """Get battery percentage"""
        if self.battery_state is None:
            return None
        
        # MAVROS reports percentage as 0-1 float
        if 0 <= self.battery_state.percentage <= 1.0:
            return self.battery_state.percentage * 100.0
        elif 0 < self.battery_state.percentage <= 100:
            return self.battery_state.percentage
        return None
    
    def get_average_current(self):
        """Get average current draw"""
        if not self.current_history:
            return self.cruise_current if self.sitl_mode else None
        return sum(self.current_history) / len(self.current_history)
    
    def calculate_distance_to_home(self):
        """Calculate distance to home position"""
        if self.home_position is None or self.global_position is None:
            return None
        
        lat1, lon1, alt1 = self.home_position
        lat2 = self.global_position.latitude
        lon2 = self.global_position.longitude
        
        # Haversine formula
        R = 6371000  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_phi / 2) ** 2 + 
             math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c
    
    def calculate_return_time(self):
        """Calculate time needed to return home"""
        distance = self.calculate_distance_to_home()
        if distance is None:
            return None
        
        # Account for headwind and safety margin
        effective_speed = self.cruise_speed * (1 - self.safety_margin)
        return distance / effective_speed
    
    def calculate_remaining_flight_time(self):
        """Calculate remaining flight time based on battery"""
        percentage = self.get_battery_percentage()
        avg_current = self.get_average_current()
        
        if percentage is None or avg_current is None or avg_current <= 0:
            return None
        
        if self.battery_state and self.battery_state.capacity > 0:
            capacity = self.battery_state.capacity
        else:
            # Estimate capacity (typical fixed-wing: 4000-10000 mAh)
            capacity = 5000  # mAh default estimate
        
        remaining_mah = (percentage / 100.0) * capacity
        remaining_hours = remaining_mah / (avg_current * 1000)
        return remaining_hours * 3600  # Convert to seconds
    
    def trigger_rtl(self, reason: str):
        """Trigger RTL mode"""
        if self.rtl_triggered:
            return
        
        self.get_logger().warn(f'TRIGGERING RTL: {reason}')
        
        # Publish alert
        alert = String()
        alert.data = json.dumps({
            'type': 'RTL_TRIGGERED',
            'reason': reason,
            'timestamp': time.time()
        })
        self.alert_pub.publish(alert)
        
        if not self.enable_rtl:
            self.get_logger().info('RTL trigger disabled - monitoring only')
            return
        
        # Call set_mode service
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            request = SetMode.Request()
            request.custom_mode = 'RTL'
            
            future = self.set_mode_client.call_async(request)
            future.add_done_callback(self.rtl_callback)
            self.rtl_triggered = True
        else:
            self.get_logger().error('Set mode service not available!')
    
    def rtl_callback(self, future):
        """Handle RTL service response"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('RTL command accepted')
            else:
                self.get_logger().error('RTL command rejected')
        except Exception as e:
            self.get_logger().error(f'RTL service call failed: {e}')
    
    def check_battery(self):
        """Main battery check logic"""
        with self.lock:
            # Skip if not armed
            if not self.is_armed():
                return
            
            # Skip if in grace period
            if self.is_in_grace_period():
                return
            
            # Skip if already in RTL or LAND
            mode = self.get_current_mode()
            if mode in ['RTL', 'LAND', 'QLAND']:
                return
            
            # Get battery percentage
            percentage = self.get_battery_percentage()
            if percentage is None:
                return
            
            # Check voltage per cell
            if self.battery_state and self.battery_state.voltage > 0:
                voltage_per_cell = self.battery_state.voltage / self.battery_cells
                min_voltage = self.min_cell_voltage * self.battery_cells
                
                if self.battery_state.voltage < min_voltage:
                    self.trigger_rtl(
                        f'Low voltage: {self.battery_state.voltage:.1f}V '
                        f'(min: {min_voltage:.1f}V)'
                    )
                    return
            
            # Check critical battery level
            if percentage <= self.critical_battery:
                self.trigger_rtl(f'Critical battery: {percentage:.1f}%')
                return
            
            # Check if enough time to return
            remaining_time = self.calculate_remaining_flight_time()
            return_time = self.calculate_return_time()
            
            if remaining_time and return_time:
                if remaining_time < return_time * (1 + self.safety_margin):
                    self.trigger_rtl(
                        f'Insufficient time to return: '
                        f'{remaining_time:.0f}s remaining, '
                        f'{return_time:.0f}s needed'
                    )
                    return
            
            # Log status periodically
            current_time = time.time()
            if current_time - self.last_log_time > 10:
                distance = self.calculate_distance_to_home()
                self.get_logger().info(
                    f'Battery: {percentage:.1f}% | '
                    f'Mode: {mode} | '
                    f'Distance: {distance:.0f}m' if distance else f'Battery: {percentage:.1f}%'
                )
                self.last_log_time = current_time
    
    def publish_status(self):
        """Publish power status"""
        with self.lock:
            percentage = self.get_battery_percentage()
            
            status = {
                'timestamp': time.time(),
                'battery_percentage': percentage,
                'voltage': self.battery_state.voltage if self.battery_state else None,
                'current': self.get_average_current(),
                'armed': self.is_armed(),
                'mode': self.get_current_mode(),
                'flight_time': self.get_flight_time(),
                'distance_to_home': self.calculate_distance_to_home(),
                'remaining_flight_time': self.calculate_remaining_flight_time(),
                'return_time_needed': self.calculate_return_time(),
                'rtl_triggered': self.rtl_triggered,
                'mission_count': self.mission_count
            }
            
            msg = String()
            msg.data = json.dumps(status, default=str)
            self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PowerMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Power Monitor')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
