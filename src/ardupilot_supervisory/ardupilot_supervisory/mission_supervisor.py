#!/usr/bin/env python3
"""
MISSION SUPERVISOR NODE - ArduPilot MAVROS2 Version
Monitors mission execution and handles waypoint tracking.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import (
    State, WaypointList, WaypointReached, 
    HomePosition, VfrHud
)
from mavros_msgs.srv import SetMode, WaypointPull
from std_msgs.msg import String
from threading import Lock
from enum import Enum
import time
import json
import math


class MissionState(Enum):
    IDLE = 0
    ARMED = 1
    TAKEOFF = 2
    MISSION = 3
    LOITER = 4
    RTL = 5
    LANDING = 6
    DISARMED = 7


class MissionSupervisor(Node):
    def __init__(self):
        super().__init__('mission_supervisor')
        
        # Declare parameters
        self.declare_parameter('mission_timeout', 3600.0)
        self.declare_parameter('waypoint_timeout', 300.0)
        self.declare_parameter('stuck_detection_radius', 50.0)
        self.declare_parameter('stuck_detection_time', 60.0)
        self.declare_parameter('check_interval', 3.0)
        self.declare_parameter('startup_grace_period', 30.0)
        self.declare_parameter('mission_start_grace', 45.0)
        self.declare_parameter('min_altitude_for_actions', 20.0)
        self.declare_parameter('sitl_mode', True)
        self.declare_parameter('enable_stuck_detection', True)
        
        # Get parameters
        self.mission_timeout = self.get_parameter('mission_timeout').value
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value
        self.stuck_radius = self.get_parameter('stuck_detection_radius').value
        self.stuck_time = self.get_parameter('stuck_detection_time').value
        self.check_interval = self.get_parameter('check_interval').value
        self.startup_grace = self.get_parameter('startup_grace_period').value
        self.mission_start_grace = self.get_parameter('mission_start_grace').value
        self.min_altitude = self.get_parameter('min_altitude_for_actions').value
        self.sitl_mode = self.get_parameter('sitl_mode').value
        self.enable_stuck = self.get_parameter('enable_stuck_detection').value
        
        # State tracking
        self.mission_state = MissionState.IDLE
        self.vehicle_state = None
        self.global_position = None
        self.local_position = None
        self.home_position = None
        self.waypoints = []
        self.vfr_hud = None
        
        # Mission tracking
        self.current_waypoint = 0
        self.total_waypoints = 0
        self.mission_start_time = None
        self.waypoint_start_time = None
        self.mission_active = False
        self.mission_count = 0
        
        # Position history for stuck detection
        self.position_history = []
        self.last_position_check = 0
        self.stuck_detected = False
        
        # Mode tracking
        self.last_mode = None
        self.mode_change_time = None
        
        # Statistics
        self.stats = {
            'total_missions': 0,
            'successful_missions': 0,
            'failed_missions': 0,
            'waypoints_reached': 0,
            'stuck_events': 0
        }
        
        self.lock = Lock()
        self.node_start_time = time.time()
        self.last_status_log = 0
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, '/supervisory/mission_status', 10)
        self.waypoint_pub = self.create_publisher(
            String, '/supervisory/current_waypoint', 10)
            
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
            State, '/mavros/state',
            self.state_callback, mavros_qos_rel_tra)
        
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.position_callback, mavros_qos_best)
        
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.local_position_callback, mavros_qos_best)
        
        self.create_subscription(
            WaypointList, '/mavros/mission/waypoints',
            self.waypoints_callback, mavros_qos_rel_tra)
        
        self.create_subscription(
            WaypointReached, '/mavros/mission/reached',
            self.waypoint_reached_callback, mavros_qos_rel_tra)
        
        self.create_subscription(
            HomePosition, '/mavros/home_position/home',
            self.home_callback, mavros_qos_rel_tra)
        
        self.create_subscription(
            VfrHud, '/mavros/vfr_hud',
            self.vfr_callback, mavros_qos_rel_vol)
        
        # Services
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.wp_pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')
        
        # Timers
        self.create_timer(self.check_interval, self.supervise_mission)
        self.create_timer(1.0, self.publish_status)
        
        self.print_startup_info()
    
    def print_startup_info(self):
        """Print startup configuration"""
        mode_str = "SITL" if self.sitl_mode else "HARDWARE"
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'MISSION SUPERVISOR - MAVROS2 VERSION ({mode_str})')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Mission Settings:')
        self.get_logger().info(f'  Mission timeout: {self.mission_timeout/60:.1f} min')
        self.get_logger().info(f'  Waypoint timeout: {self.waypoint_timeout}s')
        self.get_logger().info(f'Stuck Detection:')
        self.get_logger().info(f'  Enabled: {self.enable_stuck}')
        self.get_logger().info(f'  Radius: {self.stuck_radius}m')
        self.get_logger().info(f'  Time: {self.stuck_time}s')
        self.get_logger().info('=' * 60)
    
    def state_callback(self, msg: State):
        """Process vehicle state"""
        with self.lock:
            prev_mode = self.last_mode
            prev_armed = self.vehicle_state.armed if self.vehicle_state else False
            
            self.vehicle_state = msg
            current_mode = msg.mode
            
            # Handle arming state changes
            if msg.armed and not prev_armed:
                self.handle_arm()
            elif not msg.armed and prev_armed:
                self.handle_disarm()
            
            # Handle mode changes
            if prev_mode != current_mode:
                self.handle_mode_change(prev_mode, current_mode)
                self.last_mode = current_mode
    
    def position_callback(self, msg: NavSatFix):
        """Process GPS position"""
        with self.lock:
            self.global_position = msg
            
            # Track position for stuck detection
            if self.enable_stuck and self.is_armed() and self.mission_active:
                current_time = time.time()
                if current_time - self.last_position_check > 5.0:
                    self.position_history.append({
                        'time': current_time,
                        'lat': msg.latitude,
                        'lon': msg.longitude
                    })
                    
                    # Keep last 20 positions
                    if len(self.position_history) > 20:
                        self.position_history.pop(0)
                    
                    self.last_position_check = current_time
                    self.check_stuck()
    
    def local_position_callback(self, msg: PoseStamped):
        """Process local position"""
        with self.lock:
            self.local_position = msg
    
    def waypoints_callback(self, msg: WaypointList):
        """Process waypoint list"""
        with self.lock:
            self.waypoints = msg.waypoints
            self.total_waypoints = len(msg.waypoints)
            self.current_waypoint = msg.current_seq
            
            if self.total_waypoints > 0:
                self.get_logger().info(
                    f'Waypoints updated: {self.total_waypoints} waypoints, '
                    f'current: {self.current_waypoint}'
                )
    
    def waypoint_reached_callback(self, msg: WaypointReached):
        """Handle waypoint reached event"""
        with self.lock:
            self.current_waypoint = msg.wp_seq
            self.waypoint_start_time = time.time()
            self.stats['waypoints_reached'] += 1
            
            self.get_logger().info(
                f'Waypoint {msg.wp_seq}/{self.total_waypoints} reached'
            )
            
            # Publish waypoint event
            wp_msg = String()
            wp_msg.data = json.dumps({
                'event': 'waypoint_reached',
                'waypoint': msg.wp_seq,
                'total': self.total_waypoints,
                'timestamp': time.time()
            })
            self.waypoint_pub.publish(wp_msg)
    
    def home_callback(self, msg: HomePosition):
        """Process home position"""
        with self.lock:
            self.home_position = msg
    
    def vfr_callback(self, msg: VfrHud):
        """Process VFR HUD data"""
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
    
    def get_altitude(self):
        """Get current altitude"""
        if self.vfr_hud:
            return self.vfr_hud.altitude
        if self.local_position:
            return self.local_position.pose.position.z
        return 0.0
    
    def handle_arm(self):
        """Handle arming event"""
        self.mission_count += 1
        self.stats['total_missions'] += 1
        self.position_history = []
        self.stuck_detected = False
        self.get_logger().info(f'ARMED - Mission #{self.mission_count}')
    
    def handle_disarm(self):
        """Handle disarming event"""
        if self.mission_active:
            # Log mission summary
            duration = time.time() - self.mission_start_time if self.mission_start_time else 0
            self.get_logger().info('=' * 40)
            self.get_logger().info('Mission Summary:')
            self.get_logger().info(f'  Duration: {duration/60:.1f} min')
            self.get_logger().info(f'  Waypoints: {self.stats["waypoints_reached"]}')
            self.get_logger().info(f'  Stuck events: {self.stats["stuck_events"]}')
            self.get_logger().info('=' * 40)
        
        self.mission_active = False
        self.mission_start_time = None
        self.mission_state = MissionState.DISARMED
        self.get_logger().info('DISARMED')
    
    def handle_mode_change(self, prev_mode, current_mode):
        """Handle flight mode change"""
        self.get_logger().info(f'Mode: {prev_mode} → {current_mode}')
        self.mode_change_time = time.time()
        
        # Update mission state based on mode
        mode_mapping = {
            'AUTO': MissionState.MISSION,
            'GUIDED': MissionState.MISSION,
            'RTL': MissionState.RTL,
            'LOITER': MissionState.LOITER,
            'LAND': MissionState.LANDING,
            'TAKEOFF': MissionState.TAKEOFF,
            'MANUAL': MissionState.IDLE,
            'STABILIZE': MissionState.IDLE,
            'FBWA': MissionState.IDLE,
            'FBWB': MissionState.IDLE,
        }
        
        self.mission_state = mode_mapping.get(current_mode, MissionState.IDLE)
        
        # Detect mission start
        if current_mode == 'AUTO' and not self.mission_active:
            self.mission_active = True
            self.mission_start_time = time.time()
            self.waypoint_start_time = time.time()
            self.get_logger().info('Mission started - AUTO mode engaged')
        
        # Detect mission end
        if prev_mode == 'AUTO' and current_mode in ['LOITER', 'RTL', 'LAND']:
            self.get_logger().info(f'Mission ended - switched to {current_mode}')
    
    def is_in_grace_period(self):
        """Check if in any grace period"""
        current_time = time.time()
        
        if (current_time - self.node_start_time) < self.startup_grace:
            return True
        
        if self.mission_start_time:
            if (current_time - self.mission_start_time) < self.mission_start_grace:
                return True
        
        return False
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two coordinates"""
        R = 6371000  # Earth radius
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_phi/2)**2 + 
             math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def check_stuck(self):
        """Check if vehicle is stuck"""
        if not self.position_history or len(self.position_history) < 5:
            return
        
        current_time = time.time()
        
        # Find positions from stuck_time seconds ago
        old_positions = [p for p in self.position_history 
                        if current_time - p['time'] > self.stuck_time]
        
        if not old_positions:
            return
        
        old_pos = old_positions[0]
        current_pos = self.position_history[-1]
        
        distance = self.calculate_distance(
            old_pos['lat'], old_pos['lon'],
            current_pos['lat'], current_pos['lon']
        )
        
        # Relaxed threshold for SITL
        threshold = self.stuck_radius * 2 if self.sitl_mode else self.stuck_radius
        
        if distance < threshold:
            if not self.stuck_detected:
                self.stuck_detected = True
                self.stats['stuck_events'] += 1
                self.get_logger().warn(
                    f'Stuck detected! Moved only {distance:.1f}m in {self.stuck_time}s'
                )
        else:
            self.stuck_detected = False
    
    def trigger_rtl(self, reason: str):
        """Trigger RTL mode"""
        current_mode = self.get_current_mode()
        if current_mode in ['RTL', 'LAND']:
            return
        
        self.get_logger().warn(f'Triggering RTL: {reason}')
        
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            request = SetMode.Request()
            request.custom_mode = 'RTL'
            self.set_mode_client.call_async(request)
    
    def supervise_mission(self):
        """Main mission supervision logic"""
        with self.lock:
            # Skip if not armed or no mission active
            if not self.is_armed() or not self.mission_active:
                return
            
            # Skip during grace period
            if self.is_in_grace_period():
                return
            
            current_mode = self.get_current_mode()
            current_time = time.time()
            
            # Skip if already in safe mode
            if current_mode in ['RTL', 'LAND']:
                return
            
            # Check altitude
            altitude = self.get_altitude()
            if altitude < self.min_altitude:
                return
            
            # Check mission timeout
            if self.mission_start_time:
                mission_duration = current_time - self.mission_start_time
                timeout = self.mission_timeout * 2 if self.sitl_mode else self.mission_timeout
                
                if mission_duration > timeout:
                    self.get_logger().error(f'Mission timeout: {mission_duration/60:.1f} min')
                    self.trigger_rtl('Mission timeout')
                    return
            
            # Check waypoint timeout
            if self.waypoint_start_time and current_mode == 'AUTO':
                wp_duration = current_time - self.waypoint_start_time
                wp_timeout = self.waypoint_timeout * 2 if self.sitl_mode else self.waypoint_timeout
                
                if wp_duration > wp_timeout:
                    self.get_logger().warn(f'Waypoint timeout: {wp_duration:.0f}s')
                    if not self.sitl_mode:
                        self.trigger_rtl(f'Waypoint {self.current_waypoint} timeout')
                    return
            
            # Log status periodically
            if current_time - self.last_status_log > 10:
                self.get_logger().info(
                    f'Mission: WP {self.current_waypoint}/{self.total_waypoints}, '
                    f'Alt: {altitude:.1f}m, Mode: {current_mode}'
                )
                self.last_status_log = current_time
    
    def publish_status(self):
        """Publish mission status"""
        with self.lock:
            mission_time = 0
            if self.mission_start_time:
                mission_time = time.time() - self.mission_start_time
            
            status = {
                'timestamp': time.time(),
                'state': self.mission_state.name,
                'mode': self.get_current_mode(),
                'armed': self.is_armed(),
                'mission_active': self.mission_active,
                'current_waypoint': self.current_waypoint,
                'total_waypoints': self.total_waypoints,
                'mission_time': mission_time,
                'altitude': self.get_altitude(),
                'stuck_detected': self.stuck_detected,
                'mission_count': self.mission_count,
                'statistics': self.stats
            }
            
            msg = String()
            msg.data = json.dumps(status, default=str)
            self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Mission Supervisor')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
