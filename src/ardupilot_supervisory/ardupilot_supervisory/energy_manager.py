#!/usr/bin/env python3
"""
ENERGY MANAGER NODE - ArduPilot MAVROS2 Version
Calculates safe return range and energy status.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState, NavSatFix
from mavros_msgs.msg import State, VfrHud, HomePosition
from std_msgs.msg import String, Bool
from threading import Lock
import time
import json
import math


class EnergyManager(Node):
    def __init__(self):
        super().__init__('energy_manager')
        
        # Declare parameters
        self.declare_parameter('cruise_speed', 18.0)
        self.declare_parameter('climb_rate', 3.0)
        self.declare_parameter('descent_rate', 2.0)
        self.declare_parameter('loiter_radius', 80.0)
        self.declare_parameter('reserve_percentage', 20.0)
        self.declare_parameter('headwind_margin', 5.0)
        self.declare_parameter('calculation_interval', 5.0)
        self.declare_parameter('sitl_mode', True)
        
        # Get parameters
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.climb_rate = self.get_parameter('climb_rate').value
        self.descent_rate = self.get_parameter('descent_rate').value
        self.loiter_radius = self.get_parameter('loiter_radius').value
        self.reserve_pct = self.get_parameter('reserve_percentage').value
        self.headwind = self.get_parameter('headwind_margin').value
        self.calc_interval = self.get_parameter('calculation_interval').value
        self.sitl_mode = self.get_parameter('sitl_mode').value
        
        # State
        self.battery_state = None
        self.vehicle_state = None
        self.global_position = None
        self.home_position = None
        self.vfr_hud = None
        self.lock = Lock()
        
        # Calculations
        self.max_range = 0
        self.safe_return_range = 0
        self.estimated_return_time = 0
        self.safe_to_continue = True
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, '/supervisory/energy_status', 10)
        self.safe_return_pub = self.create_publisher(
            Bool, '/supervisory/safe_return_available', 10)
        
        
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
            HomePosition, '/mavros/home_position/home',
            self.home_callback, mavros_qos_rel_tra)
        self.create_subscription(
            VfrHud, '/mavros/vfr_hud',
            self.vfr_callback, mavros_qos_rel_vol)
        
        # Timer
        self.create_timer(self.calc_interval, self.calculate_energy)
        
        self.get_logger().info('Energy Manager started')
    
    def battery_callback(self, msg: BatteryState):
        with self.lock:
            self.battery_state = msg
    
    def state_callback(self, msg: State):
        with self.lock:
            self.vehicle_state = msg
    
    def position_callback(self, msg: NavSatFix):
        with self.lock:
            self.global_position = msg
    
    def home_callback(self, msg: HomePosition):
        with self.lock:
            self.home_position = msg
    
    def vfr_callback(self, msg: VfrHud):
        with self.lock:
            self.vfr_hud = msg
    
    def get_battery_percentage(self):
        if self.battery_state is None:
            return None
        pct = self.battery_state.percentage
        return pct * 100 if pct <= 1.0 else pct
    
    def calculate_distance_to_home(self):
        if self.home_position is None or self.global_position is None:
            return None
        
        lat1 = self.home_position.geo.latitude
        lon1 = self.home_position.geo.longitude
        lat2 = self.global_position.latitude
        lon2 = self.global_position.longitude
        
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_phi/2)**2 + 
             math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def calculate_energy(self):
        with self.lock:
            battery_pct = self.get_battery_percentage()
            if battery_pct is None:
                return
            
            # Usable battery (minus reserve)
            usable_pct = max(0, battery_pct - self.reserve_pct)
            
            # Estimate flight time remaining (simplified)
            # Assume 1% battery = ~30 seconds of flight
            flight_time_remaining = usable_pct * 30  # seconds
            
            # Calculate range with headwind consideration
            effective_speed = max(1, self.cruise_speed - self.headwind)
            self.max_range = flight_time_remaining * effective_speed
            
            # Safe return range (half of max, accounting for round trip)
            self.safe_return_range = self.max_range / 2
            
            # Distance to home
            distance_home = self.calculate_distance_to_home()
            
            # Calculate return time
            if distance_home:
                self.estimated_return_time = distance_home / effective_speed
                # Add loiter approach time
                self.estimated_return_time += (2 * math.pi * self.loiter_radius) / self.cruise_speed
                
                # Check if safe to continue
                self.safe_to_continue = distance_home < self.safe_return_range
            else:
                self.safe_to_continue = True
            
            # Publish status
            status = {
                'timestamp': time.time(),
                'battery_percentage': battery_pct,
                'usable_percentage': usable_pct,
                'max_range_m': self.max_range,
                'safe_return_range_m': self.safe_return_range,
                'distance_to_home_m': distance_home,
                'estimated_return_time_s': self.estimated_return_time,
                'safe_to_continue': self.safe_to_continue,
                'groundspeed': self.vfr_hud.groundspeed if self.vfr_hud else None
            }
            
            msg = String()
            msg.data = json.dumps(status, default=str)
            self.status_pub.publish(msg)
            
            safe_msg = Bool()
            safe_msg.data = self.safe_to_continue
            self.safe_return_pub.publish(safe_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnergyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Energy Manager')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
