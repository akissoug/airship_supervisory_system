#!/usr/bin/env python3
"""
TELEMETRY HANDLER NODE - ArduPilot MAVROS2 Version
Aggregates telemetry data and provides logging.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, VfrHud
from std_msgs.msg import String
from threading import Lock
from collections import deque
from datetime import datetime
import time
import json
import os


class TelemetryHandler(Node):
    def __init__(self):
        super().__init__('telemetry_handler')
        
        # Parameters
        self.declare_parameter('log_to_file', True)
        self.declare_parameter('log_directory', '/home/user/flight_logs')
        self.declare_parameter('telemetry_rate', 1.0)
        self.declare_parameter('position_history_size', 1000)
        self.declare_parameter('battery_history_size', 500)
        self.declare_parameter('sitl_mode', True)
        
        self.log_to_file = self.get_parameter('log_to_file').value
        self.log_dir = self.get_parameter('log_directory').value
        self.telem_rate = self.get_parameter('telemetry_rate').value
        self.pos_history_size = self.get_parameter('position_history_size').value
        self.batt_history_size = self.get_parameter('battery_history_size').value
        self.sitl_mode = self.get_parameter('sitl_mode').value
        
        # Data storage
        self.telemetry = {
            'battery': None,
            'state': None,
            'position': None,
            'local_position': None,
            'velocity': None,
            'vfr_hud': None,
            'imu': None
        }
        
        # History
        self.position_history = deque(maxlen=self.pos_history_size)
        self.battery_history = deque(maxlen=self.batt_history_size)
        self.airspeed_history = deque(maxlen=100)
        
        # Statistics
        self.flight_stats = {
            'start_time': None,
            'max_altitude': 0,
            'max_speed': 0,
            'total_distance': 0,
            'min_battery': 100
        }
        
        self.lock = Lock()
        self.log_file = None
        
        # Setup logging
        if self.log_to_file:
            self.setup_logging()
        
        # Publishers
        self.telem_pub = self.create_publisher(
            String, '/supervisory/telemetry', 10)
            
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
        
        # Subscribers
        self.create_subscription(BatteryState, '/mavros/battery',
                                self.battery_callback, mavros_qos_best)
        self.create_subscription(State, '/mavros/state',
                                self.state_callback, mavros_qos_rel_tra)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                self.position_callback, mavros_qos_best)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                self.local_position_callback, mavros_qos_best)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local',
                                self.velocity_callback, mavros_qos_best)
        self.create_subscription(VfrHud, '/mavros/vfr_hud',
                                self.vfr_callback, mavros_qos_rel_vol)
        self.create_subscription(Imu, '/mavros/imu/data',
                                self.imu_callback, mavros_qos_best)
        
        # Timer
        self.create_timer(1.0 / self.telem_rate, self.publish_telemetry)
        
        self.get_logger().info('Telemetry Handler started')
    
    def setup_logging(self):
        """Setup flight log file"""
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.log_filename = os.path.join(
                self.log_dir, f'flight_log_{timestamp}.json')
            self.get_logger().info(f'Logging to: {self.log_filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to setup logging: {e}')
            self.log_to_file = False
    
    def battery_callback(self, msg: BatteryState):
        with self.lock:
            self.telemetry['battery'] = msg
            pct = msg.percentage * 100 if msg.percentage <= 1.0 else msg.percentage
            self.battery_history.append({
                'time': time.time(),
                'percentage': pct,
                'voltage': msg.voltage
            })
            self.flight_stats['min_battery'] = min(
                self.flight_stats['min_battery'], pct)
    
    def state_callback(self, msg: State):
        with self.lock:
            prev_armed = self.telemetry['state'].armed if self.telemetry['state'] else False
            self.telemetry['state'] = msg
            
            if msg.armed and not prev_armed:
                self.flight_stats['start_time'] = time.time()
    
    def position_callback(self, msg: NavSatFix):
        with self.lock:
            self.telemetry['position'] = msg
            self.position_history.append({
                'time': time.time(),
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude
            })
    
    def local_position_callback(self, msg: PoseStamped):
        with self.lock:
            self.telemetry['local_position'] = msg
            alt = msg.pose.position.z
            self.flight_stats['max_altitude'] = max(
                self.flight_stats['max_altitude'], alt)
    
    def velocity_callback(self, msg: TwistStamped):
        with self.lock:
            self.telemetry['velocity'] = msg
    
    def vfr_callback(self, msg: VfrHud):
        with self.lock:
            self.telemetry['vfr_hud'] = msg
            self.flight_stats['max_speed'] = max(
                self.flight_stats['max_speed'], msg.groundspeed)
            self.airspeed_history.append({
                'time': time.time(),
                'airspeed': msg.airspeed,
                'groundspeed': msg.groundspeed
            })
    
    def imu_callback(self, msg: Imu):
        with self.lock:
            self.telemetry['imu'] = msg
    
    def publish_telemetry(self):
        with self.lock:
            telem_data = {
                'timestamp': time.time(),
                'battery': {
                    'percentage': (self.telemetry['battery'].percentage * 100 
                                  if self.telemetry['battery'] else None),
                    'voltage': (self.telemetry['battery'].voltage 
                               if self.telemetry['battery'] else None)
                },
                'state': {
                    'armed': (self.telemetry['state'].armed 
                             if self.telemetry['state'] else False),
                    'mode': (self.telemetry['state'].mode 
                            if self.telemetry['state'] else 'UNKNOWN')
                },
                'position': {
                    'lat': (self.telemetry['position'].latitude 
                           if self.telemetry['position'] else None),
                    'lon': (self.telemetry['position'].longitude 
                           if self.telemetry['position'] else None),
                    'alt': (self.telemetry['position'].altitude 
                           if self.telemetry['position'] else None)
                },
                'vfr': {
                    'airspeed': (self.telemetry['vfr_hud'].airspeed 
                                if self.telemetry['vfr_hud'] else None),
                    'groundspeed': (self.telemetry['vfr_hud'].groundspeed 
                                   if self.telemetry['vfr_hud'] else None),
                    'heading': (self.telemetry['vfr_hud'].heading 
                               if self.telemetry['vfr_hud'] else None)
                },
                'statistics': self.flight_stats,
                'history_points': len(self.position_history)
            }
            
            msg = String()
            msg.data = json.dumps(telem_data, default=str)
            self.telem_pub.publish(msg)
            
            if self.log_to_file and hasattr(self, 'log_filename'):
                try:
                    with open(self.log_filename, 'a') as f:
                        f.write(json.dumps(telem_data, default=str) + '\n')
                except Exception as e:
                    pass  # Silent fail for logging


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Telemetry Handler')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
