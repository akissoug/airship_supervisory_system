#!/usr/bin/env python3
"""
EMERGENCY MANAGER NODE - ArduPilot MAVROS2 Version
Coordinates emergency responses and failsafe actions.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState, NavSatFix
from mavros_msgs.msg import State, VfrHud
from mavros_msgs.srv import SetMode, CommandBool, CommandLong
from std_msgs.msg import String, Bool
from threading import Lock
from enum import Enum
import time
import json


class EmergencyType(Enum):
    NONE = 0
    LOW_BATTERY = 1
    CRITICAL_BATTERY = 2
    SENSOR_FAILURE = 3
    GPS_LOSS = 4
    COMMUNICATION_LOSS = 5
    GEOFENCE_BREACH = 6
    MANUAL_TRIGGER = 7


class EmergencyManager(Node):
    def __init__(self):
        super().__init__('emergency_manager')
        
        # Parameters
        self.declare_parameter('enable_auto_emergency', True)
        self.declare_parameter('emergency_loiter_altitude', 100.0)
        self.declare_parameter('emergency_loiter_radius', 150.0)
        self.declare_parameter('emergency_timeout', 300.0)
        self.declare_parameter('max_emergency_retries', 3)
        self.declare_parameter('command_cooldown', 15.0)
        self.declare_parameter('sitl_mode', True)
        
        self.enable_auto = self.get_parameter('enable_auto_emergency').value
        self.emergency_alt = self.get_parameter('emergency_loiter_altitude').value
        self.emergency_radius = self.get_parameter('emergency_loiter_radius').value
        self.emergency_timeout = self.get_parameter('emergency_timeout').value
        self.max_retries = self.get_parameter('max_emergency_retries').value
        self.command_cooldown = self.get_parameter('command_cooldown').value
        self.sitl_mode = self.get_parameter('sitl_mode').value
        
        # State
        self.vehicle_state = None
        self.battery_state = None
        self.global_position = None
        self.vfr_hud = None
        
        self.emergency_active = False
        self.emergency_type = EmergencyType.NONE
        self.emergency_start_time = None
        self.emergency_retries = 0
        self.last_command_time = 0
        
        self.lock = Lock()
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, '/supervisory/emergency_status', 10)
        self.alert_pub = self.create_publisher(
            String, '/supervisory/emergency_alert', 10)
        
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
            State, '/mavros/state', self.state_callback, mavros_qos_rel_tra)
        self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_callback, mavros_qos_best)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', 
            self.position_callback, mavros_qos_best)
        self.create_subscription(
            VfrHud, '/mavros/vfr_hud', self.vfr_callback, mavros_qos_rel_vol)
        
        # Subscribe to other node alerts
        self.create_subscription(
            String, '/supervisory/power_alert', 
            self.power_alert_callback, 10)
        self.create_subscription(
            String, '/supervisory/active_faults', 
            self.fault_alert_callback, 10)
        self.create_subscription(
            Bool, '/supervisory/safe_return_available',
            self.safe_return_callback, 10)
        
        # Services
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        # Timer
        self.create_timer(1.0, self.check_emergency)
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Emergency Manager started')
        self.get_logger().info(f'Auto emergency: {self.enable_auto}')
    
    def state_callback(self, msg: State):
        with self.lock:
            self.vehicle_state = msg
    
    def battery_callback(self, msg: BatteryState):
        with self.lock:
            self.battery_state = msg
    
    def position_callback(self, msg: NavSatFix):
        with self.lock:
            self.global_position = msg
    
    def vfr_callback(self, msg: VfrHud):
        with self.lock:
            self.vfr_hud = msg
    
    def power_alert_callback(self, msg: String):
        """Handle power monitor alerts"""
        try:
            alert = json.loads(msg.data)
            if alert.get('type') == 'RTL_TRIGGERED':
                self.trigger_emergency(
                    EmergencyType.LOW_BATTERY,
                    alert.get('reason', 'Low battery')
                )
        except json.JSONDecodeError:
            pass
    
    def fault_alert_callback(self, msg: String):
        """Handle fault detector alerts"""
        try:
            data = json.loads(msg.data)
            faults = data.get('faults', [])
            
            if 'GPS_UNHEALTHY' in faults or 'GPS_TIMEOUT' in faults:
                self.trigger_emergency(EmergencyType.GPS_LOSS, 'GPS failure')
            elif any('UNHEALTHY' in f for f in faults):
                self.trigger_emergency(EmergencyType.SENSOR_FAILURE, 'Sensor failure')
        except json.JSONDecodeError:
            pass
    
    def safe_return_callback(self, msg: Bool):
        """Handle energy manager safe return status"""
        if not msg.data and self.is_armed():
            self.get_logger().warn('Safe return no longer available!')
    
    def is_armed(self):
        return self.vehicle_state is not None and self.vehicle_state.armed
    
    def get_current_mode(self):
        if self.vehicle_state is None:
            return "UNKNOWN"
        return self.vehicle_state.mode
    
    def trigger_emergency(self, emergency_type: EmergencyType, reason: str):
        """Trigger emergency response"""
        with self.lock:
            if self.emergency_active:
                return
            
            if not self.enable_auto:
                self.get_logger().warn(f'Emergency trigger (disabled): {reason}')
                return
            
            self.get_logger().error(f'EMERGENCY: {emergency_type.name} - {reason}')
            
            self.emergency_active = True
            self.emergency_type = emergency_type
            self.emergency_start_time = time.time()
            
            # Publish alert
            alert = String()
            alert.data = json.dumps({
                'type': 'EMERGENCY',
                'emergency_type': emergency_type.name,
                'reason': reason,
                'timestamp': time.time()
            })
            self.alert_pub.publish(alert)
            
            # Execute emergency action
            self.execute_emergency_action()
    
    def execute_emergency_action(self):
        """Execute appropriate emergency action"""
        if time.time() - self.last_command_time < self.command_cooldown:
            return
        
        current_mode = self.get_current_mode()
        
        # Already in safe mode
        if current_mode in ['RTL', 'LAND', 'QLAND']:
            self.get_logger().info(f'Already in {current_mode} mode')
            return
        
        # Determine best action based on emergency type
        if self.emergency_type == EmergencyType.GPS_LOSS:
            # No GPS - can't RTL, try to maintain altitude
            self.get_logger().warn('GPS loss - maintaining current heading')
            # Could switch to FBWA or similar stabilized mode
            self.set_mode('FBWA')
        else:
            # Default: RTL
            self.set_mode('RTL')
        
        self.last_command_time = time.time()
        self.emergency_retries += 1
    
    def set_mode(self, mode: str):
        """Set flight mode"""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Set mode service not available')
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(
            lambda f: self.mode_callback(f, mode))
        return True
    
    def mode_callback(self, future, mode: str):
        """Handle mode change response"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'Mode change to {mode} accepted')
            else:
                self.get_logger().error(f'Mode change to {mode} rejected')
        except Exception as e:
            self.get_logger().error(f'Mode service call failed: {e}')
    
    def check_emergency(self):
        """Check and manage emergency state"""
        with self.lock:
            if not self.emergency_active:
                return
            
            current_mode = self.get_current_mode()
            
            # Check if emergency resolved
            if current_mode in ['RTL', 'LAND', 'QLAND'] or not self.is_armed():
                elapsed = time.time() - self.emergency_start_time
                if elapsed > 30:  # Wait for mode to stabilize
                    self.get_logger().info('Emergency handled - vehicle in safe mode')
                    self.emergency_active = False
                    self.emergency_type = EmergencyType.NONE
                return
            
            # Check timeout
            if time.time() - self.emergency_start_time > self.emergency_timeout:
                self.get_logger().error('Emergency timeout - escalating')
                # Could trigger more aggressive action here
                self.emergency_active = False
                return
            
            # Retry if needed
            if self.emergency_retries < self.max_retries:
                self.execute_emergency_action()
    
    def publish_status(self):
        """Publish emergency manager status"""
        with self.lock:
            status = {
                'timestamp': time.time(),
                'emergency_active': self.emergency_active,
                'emergency_type': self.emergency_type.name,
                'armed': self.is_armed(),
                'mode': self.get_current_mode(),
                'retries': self.emergency_retries,
                'auto_enabled': self.enable_auto
            }
            
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Emergency Manager')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
