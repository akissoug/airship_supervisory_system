#!/usr/bin/env python3
"""
FAULT DETECTOR NODE - ArduPilot MAVROS2 Version
Monitors sensor health and detects system faults.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from mavros_msgs.msg import State, VfrHud, RCIn, ExtendedState
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from threading import Lock
from collections import deque
import time
import json
import math


class FaultDetector(Node):
    def __init__(self):
        super().__init__('fault_detector')
        
        # Declare parameters
        self.declare_parameter('min_satellites', 8)
        self.declare_parameter('min_hdop', 2.5)
        self.declare_parameter('gps_timeout', 5.0)
        self.declare_parameter('max_vibration', 30.0)
        self.declare_parameter('imu_timeout', 2.0)
        self.declare_parameter('min_airspeed', 12.0)
        self.declare_parameter('max_airspeed', 35.0)
        self.declare_parameter('airspeed_timeout', 5.0)
        self.declare_parameter('check_interval', 1.0)
        self.declare_parameter('consecutive_failures', 3)
        self.declare_parameter('startup_grace_period', 30.0)
        self.declare_parameter('sitl_mode', True)
        
        # Get parameters
        self.min_satellites = self.get_parameter('min_satellites').value
        self.min_hdop = self.get_parameter('min_hdop').value
        self.gps_timeout = self.get_parameter('gps_timeout').value
        self.max_vibration = self.get_parameter('max_vibration').value
        self.imu_timeout = self.get_parameter('imu_timeout').value
        self.min_airspeed = self.get_parameter('min_airspeed').value
        self.max_airspeed = self.get_parameter('max_airspeed').value
        self.airspeed_timeout = self.get_parameter('airspeed_timeout').value
        self.check_interval = self.get_parameter('check_interval').value
        self.failure_threshold = self.get_parameter('consecutive_failures').value
        self.startup_grace = self.get_parameter('startup_grace_period').value
        self.sitl_mode = self.get_parameter('sitl_mode').value
        
        # Sensor status tracking
        self.sensors = {
            'gps': {'healthy': False, 'last_update': 0, 'failures': 0, 'data': None},
            'imu': {'healthy': False, 'last_update': 0, 'failures': 0, 'data': None},
            'mag': {'healthy': False, 'last_update': 0, 'failures': 0, 'data': None},
            'airspeed': {'healthy': True, 'last_update': 0, 'failures': 0, 'data': None},
            'rc': {'healthy': True, 'last_update': 0, 'failures': 0, 'data': None},
        }
        
        self.vehicle_state = None
        self.extended_state = None
        self.active_faults = []
        self.lock = Lock()
        self.node_start_time = time.time()
        self.last_health_log = 0
        
        # Vibration tracking
        self.vibration_history = deque(maxlen=50)
        
        # Publishers
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.fault_pub = self.create_publisher(
            String, '/supervisory/active_faults', 10)
        self.status_pub = self.create_publisher(
            String, '/supervisory/fault_status', 10)
            
            
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
            NavSatFix, '/mavros/global_position/raw/fix',
            self.gps_callback, mavros_qos_best)
        
        self.create_subscription(
            Imu, '/mavros/imu/data',
            self.imu_callback, mavros_qos_best)
        
        self.create_subscription(
            MagneticField, '/mavros/imu/mag',
            self.mag_callback, mavros_qos_best)
        
        self.create_subscription(
            State, '/mavros/state',
            self.state_callback, mavros_qos_rel_tra)
        
        self.create_subscription(
            ExtendedState, '/mavros/extended_state',
            self.extended_state_callback, mavros_qos_rel_tra)
        
        self.create_subscription(
            VfrHud, '/mavros/vfr_hud',
            self.vfr_callback, mavros_qos_rel_vol)
        
        self.create_subscription(
            RCIn, '/mavros/rc/in',
            self.rc_callback, mavros_qos_rel_vol)
        
        # Timers
        self.create_timer(self.check_interval, self.check_sensor_health)
        self.create_timer(1.0, self.publish_diagnostics)
        
        self.print_startup_info()
    
    def print_startup_info(self):
        """Print startup configuration"""
        mode_str = "SITL" if self.sitl_mode else "HARDWARE"
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'FAULT DETECTOR - MAVROS2 VERSION ({mode_str})')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'GPS Thresholds:')
        self.get_logger().info(f'  Min satellites: {self.min_satellites}')
        self.get_logger().info(f'  Max HDOP: {self.min_hdop}')
        self.get_logger().info(f'  Timeout: {self.gps_timeout}s')
        self.get_logger().info(f'Airspeed Thresholds (fixed-wing):')
        self.get_logger().info(f'  Min (stall warning): {self.min_airspeed} m/s')
        self.get_logger().info(f'  Max (overspeed): {self.max_airspeed} m/s')
        self.get_logger().info(f'Vibration max: {self.max_vibration} m/s²')
        self.get_logger().info(f'Failure threshold: {self.failure_threshold} consecutive')
        self.get_logger().info('=' * 60)
    
    def gps_callback(self, msg: NavSatFix):
        """Process GPS data"""
        with self.lock:
            self.sensors['gps']['last_update'] = time.time()
            self.sensors['gps']['data'] = msg
            
            # Check GPS fix quality
            # NavSatFix status: -1=no fix, 0=fix, 1=sbas, 2=gbas
            if msg.status.status >= 0:
                # In real hardware, check for HDOP and satellites
                # MAVROS provides this in other topics, simplified here
                self.sensors['gps']['healthy'] = True
                self.sensors['gps']['failures'] = 0
            else:
                self.sensors['gps']['healthy'] = False
    
    def imu_callback(self, msg: Imu):
        """Process IMU data"""
        with self.lock:
            self.sensors['imu']['last_update'] = time.time()
            self.sensors['imu']['data'] = msg
            
            # Calculate vibration level
            accel = msg.linear_acceleration
            vibration = math.sqrt(accel.x**2 + accel.y**2 + accel.z**2)
            self.vibration_history.append(vibration)
            
            # Check vibration levels
            if len(self.vibration_history) > 10:
                avg_vibration = sum(self.vibration_history) / len(self.vibration_history)
                # Subtract gravity (~9.81) for vibration calculation
                vibration_level = abs(avg_vibration - 9.81)
                
                if vibration_level > self.max_vibration:
                    self.sensors['imu']['healthy'] = False
                    self.get_logger().warn(f'High vibration: {vibration_level:.1f} m/s²')
                else:
                    self.sensors['imu']['healthy'] = True
                    self.sensors['imu']['failures'] = 0
            else:
                self.sensors['imu']['healthy'] = True
    
    ## To check it again 
    #def mag_callback(self, msg: MagneticField):
     #   """Process magnetometer data"""
      #  with self.lock:
       #     self.sensors['mag']['last_update'] = time.time()
      #      self.sensors['mag']['data'] = msg
            
        #    # Check for valid magnetic field readings
        #    field = msg.magnetic_field
        #    magnitude = math.sqrt(field.x**2 + field.y**2 + field.z**2)
            
        #   # Typical Earth's field: 25-65 μT
         #   if 20e-6 < magnitude < 70e-6:
         #       self.sensors['mag']['healthy'] = True
         #       self.sensors['mag']['failures'] = 0
         #   else:
          #      self.sensors['mag']['healthy'] = False
    
    def mag_callback(self, msg: MagneticField):
        with self.lock:
            self.sensors['mag']['last_update'] = time.time()
            self.sensors['mag']['data'] = msg

            f = msg.magnetic_field
            mag = math.sqrt(f.x**2 + f.y**2 + f.z**2)
  
            # Maintain a small history of magnitudes
            if not hasattr(self, "mag_history"):
                self.mag_history = deque(maxlen=30)
            self.mag_history.append(mag)

            # SITL: only require that data exists
            if self.sitl_mode:
                self.sensors['mag']['healthy'] = True
                self.sensors['mag']['failures'] = 0
                return

            # Hardware health logic:
            # 1) If we don't have enough samples yet, don't fail it
            if len(self.mag_history) < 10:
                self.sensors['mag']['healthy'] = True
                return

            # 2) Detect "stuck" sensor (magnitude nearly constant)
            span = max(self.mag_history) - min(self.mag_history)
            if span < 1.0:  # threshold in *your raw units*
                self.sensors['mag']['healthy'] = False
                return

            # 3) Detect sudden unrealistic jump (tune threshold)
            prev = list(self.mag_history)[-2]
            if abs(mag - prev) > 200000.0:  # jump threshold in raw units
                self.sensors['mag']['healthy'] = False
                return

            self.sensors['mag']['healthy'] = True
            self.sensors['mag']['failures'] = 0
    
    def state_callback(self, msg: State):
        """Process vehicle state"""
        with self.lock:
            self.vehicle_state = msg
    
    def extended_state_callback(self, msg: ExtendedState):
        """Process extended state"""
        with self.lock:
            self.extended_state = msg
    
    def vfr_callback(self, msg: VfrHud):
        """Process VFR HUD data including airspeed"""
        with self.lock:
            self.sensors['airspeed']['last_update'] = time.time()
            self.sensors['airspeed']['data'] = msg
            
            airspeed = msg.airspeed
            
            # Only check airspeed if armed and in flight
            if self.is_armed() and self.get_flight_time() > 30:
                if airspeed < self.min_airspeed:
                    self.get_logger().warn(f'Low airspeed warning: {airspeed:.1f} m/s')
                    self.sensors['airspeed']['healthy'] = False
                elif airspeed > self.max_airspeed:
                    self.get_logger().warn(f'Overspeed warning: {airspeed:.1f} m/s')
                    self.sensors['airspeed']['healthy'] = False
                else:
                    self.sensors['airspeed']['healthy'] = True
                    self.sensors['airspeed']['failures'] = 0
            else:
                self.sensors['airspeed']['healthy'] = True
    
    def rc_callback(self, msg: RCIn):
        """Process RC input"""
        with self.lock:
            self.sensors['rc']['last_update'] = time.time()
            self.sensors['rc']['data'] = msg
            
            # Check for valid RC signal
            # RSSI of 0 or very low channels count indicates loss
            if len(msg.channels) > 0 and msg.rssi > 0:
                self.sensors['rc']['healthy'] = True
                self.sensors['rc']['failures'] = 0
            else:
                self.sensors['rc']['healthy'] = False
    
    def is_armed(self):
        """Check if vehicle is armed"""
        return self.vehicle_state is not None and self.vehicle_state.armed
    
    def get_current_mode(self):
        """Get current flight mode"""
        if self.vehicle_state is None:
            return "UNKNOWN"
        return self.vehicle_state.mode
    
    def get_flight_time(self):
        """Approximate flight time since arm"""
        # This should ideally track arm time, simplified here
        return time.time() - self.node_start_time
    
    def is_in_grace_period(self):
        """Check if in startup grace period"""
        return (time.time() - self.node_start_time) < self.startup_grace
    
    def check_sensor_health(self):
        """Main sensor health check"""
        with self.lock:
            current_time = time.time()
            new_faults = []
            
            # Skip detailed checks during grace period
            if self.is_in_grace_period():
                return
            
            # Check each sensor
            for sensor_name, sensor in self.sensors.items():
                # Check for timeout
                if sensor['last_update'] > 0:
                    time_since_update = current_time - sensor['last_update']
                    
                    timeout = self.gps_timeout if sensor_name == 'gps' else self.imu_timeout
                    
                    if time_since_update > timeout:
                        sensor['healthy'] = False
                        sensor['failures'] += 1
                        
                        if sensor['failures'] >= self.failure_threshold:
                            new_faults.append(f'{sensor_name.upper()}_TIMEOUT')
                
                # Check health status
                if not sensor['healthy']:
                    sensor['failures'] += 1
                    
                    if sensor['failures'] >= self.failure_threshold:
                        new_faults.append(f'{sensor_name.upper()}_UNHEALTHY')
            
            # Update active faults
            self.active_faults = list(set(new_faults))
            
            # Log status periodically
            if current_time - self.last_health_log > 10:
                unhealthy = [s for s, d in self.sensors.items() if not d['healthy']]
                if unhealthy:
                    self.get_logger().warn(f'Unhealthy sensors: {", ".join(unhealthy)}')
                else:
                    self.get_logger().info('All sensors healthy')
                self.last_health_log = current_time
            
            # Publish active faults
            if self.active_faults:
                msg = String()
                msg.data = json.dumps({
                    'faults': self.active_faults,
                    'timestamp': current_time
                })
                self.fault_pub.publish(msg)
    
    def publish_diagnostics(self):
        """Publish diagnostic messages"""
        with self.lock:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            for sensor_name, sensor in self.sensors.items():
                status = DiagnosticStatus()
                status.name = f'fault_detector/{sensor_name}'
                
                if sensor['healthy']:
                    status.level = DiagnosticStatus.OK
                    status.message = 'Healthy'
                else:
                    status.level = DiagnosticStatus.WARN
                    status.message = f'Unhealthy (failures: {sensor["failures"]})'
                
                status.values.append(
                    KeyValue(key='last_update', 
                            value=f'{sensor["last_update"]:.1f}'))
                status.values.append(
                    KeyValue(key='failures', 
                            value=str(sensor['failures'])))
                
                diag_array.status.append(status)
            
            self.diagnostics_pub.publish(diag_array)
            
            # Publish status
            status_msg = String()
            status_msg.data = json.dumps({
                'timestamp': time.time(),
                'sensors': {k: v['healthy'] for k, v in self.sensors.items()},
                'active_faults': self.active_faults,
                'armed': self.is_armed(),
                'mode': self.get_current_mode()
            })
            self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaultDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Fault Detector')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
