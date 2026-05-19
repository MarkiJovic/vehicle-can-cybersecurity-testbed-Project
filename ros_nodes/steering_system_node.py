#!/usr/bin/env python3

import rospy
import time
import math
from std_msgs.msg import Float32, Bool, Int32
from can_msgs.msg import Frame

class SteeringSystemNode:
    def __init__(self):
        rospy.init_node('steering_system', anonymous=True)
        
        # Subscribe to CAN bus
        self.can_sub = rospy.Subscriber('/can_rx', Frame, self.can_callback)
        
        # Subscribe to vehicle speed
        self.speed_sub = rospy.Subscriber('/vehicle/speed', Float32, self.speed_callback)
        
        # Publishers
        self.steering_angle_pub = rospy.Publisher('/steering/angle', Float32, queue_size=10)
        self.steering_torque_pub = rospy.Publisher('/steering/torque', Float32, queue_size=10)
        self.steering_assist_pub = rospy.Publisher('/steering/assist_level', Float32, queue_size=10)
        self.steering_temp_pub = rospy.Publisher('/steering/temperature', Float32, queue_size=10)
        
        # CAN publisher
        self.can_pub = rospy.Publisher('/can_tx', Frame, queue_size=10)
        
        # Steering system state
        self.steering_angle = 0.0  # Degrees (-540 to +540)
        self.target_angle = 0.0
        self.vehicle_speed = 0.0  # km/h
        self.steering_torque = 0.0  # Nm
        self.steering_temp = 90.0  # Celsius
        self.power_assist_level = 0.0  # 0-100%
        
        # Steering parameters
        self.max_angle = 540.0  # 1.5 turns lock-to-lock
        self.steering_ratio = 16.0  # Steering ratio
        self.max_torque = 50.0  # Nm
        
        # Power steering parameters
        self.assist_speed_map = {
            0: 100.0,    # Full assist at standstill
            10: 80.0,     # High assist at low speed
            30: 60.0,     # Medium assist at city speed
            60: 40.0,     # Low assist at highway speed
            100: 20.0     # Minimal assist at high speed
        }
        
        rospy.loginfo("Steering System Node initialized")
        
    def can_callback(self, msg):
        """Process incoming CAN messages"""
        # Process steering command (ID 0x101)
        if msg.id == 0x101 and len(msg.data) >= 1:
            steering_byte = msg.data[0]
            self.process_steering_command(steering_byte)
            
        # Process steering mode (ID 0x106)
        elif msg.id == 0x106 and len(msg.data) >= 1:
            mode_cmd = msg.data[0]
            self.process_steering_mode(mode_cmd)
            
    def speed_callback(self, msg):
        """Process vehicle speed updates"""
        self.vehicle_speed = msg.data
        self.calculate_power_assist()
        
    def process_steering_command(self, steering_byte):
        """Process steering command from CAN"""
        # Convert byte to steering angle (0-255 -> -540 to +540 degrees)
        self.target_angle = (steering_byte / 255.0) * self.max_angle - (self.max_angle / 2.0)
        
        rospy.loginfo(f"Steering command: {steering_byte} -> {self.target_angle:.1f}°")
        
    def process_steering_mode(self, mode_cmd):
        """Process steering mode command"""
        if mode_cmd == 0x01:  # Sport mode
            # Reduce assist by 20%
            for speed in self.assist_speed_map:
                self.assist_speed_map[speed] *= 0.8
        elif mode_cmd == 0x02:  # Comfort mode
            # Increase assist by 30%
            for speed in self.assist_speed_map:
                self.assist_speed_map[speed] *= 1.3
        elif mode_cmd == 0x03:  # Normal mode
            # Reset to default
            self.assist_speed_map = {
                0: 100.0, 10: 80.0, 30: 60.0, 60: 40.0, 100: 20.0
            }
            
    def calculate_power_assist(self):
        """Calculate power steering assist based on speed"""
        # Interpolate assist level based on speed
        speeds = sorted(self.assist_speed_map.keys())
        
        if self.vehicle_speed <= speeds[0]:
            self.power_assist_level = self.assist_speed_map[speeds[0]]
        elif self.vehicle_speed >= speeds[-1]:
            self.power_assist_level = self.assist_speed_map[speeds[-1]]
        else:
            # Linear interpolation between speed points
            for i in range(len(speeds) - 1):
                if speeds[i] <= self.vehicle_speed <= speeds[i + 1]:
                    speed_ratio = (self.vehicle_speed - speeds[i]) / (speeds[i + 1] - speeds[i])
                    assist_low = self.assist_speed_map[speeds[i]]
                    assist_high = self.assist_speed_map[speeds[i + 1]]
                    self.power_assist_level = assist_low + speed_ratio * (assist_high - assist_low)
                    break
                    
    def calculate_steering_torque(self):
        """Calculate steering torque requirements"""
        # Base torque from angle difference
        angle_diff = self.target_angle - self.steering_angle
        
        # Torque increases with angle difference and decreases with speed
        base_torque = abs(angle_diff) / self.max_angle * self.max_torque
        speed_factor = max(0.2, 1.0 - self.vehicle_speed / 100.0)
        
        # Apply power assist
        assist_factor = self.power_assist_level / 100.0
        self.steering_torque = base_torque * speed_factor * (1.0 - assist_factor * 0.7)
        
        # Add realistic steering feel
        if abs(angle_diff) < 1.0:
            # Add centering force
            self.steering_torque += self.steering_angle * 0.1
            
    def calculate_steering_temperature(self):
        """Calculate steering system temperature"""
        # Temperature increases with steering activity and decreases with speed
        base_temp = 90.0
        
        # Activity factor (based on torque)
        activity_factor = (abs(self.steering_torque) / self.max_torque) ** 2
        
        # Speed cooling factor
        cooling_factor = min(self.vehicle_speed / 50.0, 1.0) * 0.1
        
        target_temp = base_temp + (activity_factor * 30.0) - (cooling_factor * 20.0)
        
        # Smooth temperature change
        temp_diff = target_temp - self.steering_temp
        self.steering_temp += temp_diff * 0.02
        
    def update_steering_angle(self):
        """Update steering angle with realistic response"""
        # Calculate angle difference
        angle_diff = self.target_angle - self.steering_angle
        
        # Speed-dependent response time
        if self.vehicle_speed < 10:
            response_rate = 0.15  # Slower at low speed (more control)
        elif self.vehicle_speed < 50:
            response_rate = 0.25  # Medium at city speed
        else:
            response_rate = 0.35  # Faster at highway speed
            
        # Apply power assist effect
        assist_boost = 1.0 + (self.power_assist_level / 100.0) * 0.5
        self.steering_angle += angle_diff * response_rate * assist_boost
        
        # Add damping to prevent oscillation
        self.steering_angle *= 0.95
        
        # Limit to max angle
        self.steering_angle = max(-self.max_angle/2, min(self.max_angle/2, self.steering_angle))
        
    def send_steering_status(self):
        """Send steering system status via CAN"""
        can_msg = Frame()
        can_msg.id = 0x205
        can_msg.dlc = 8
        can_msg.data = [
            int((self.steering_angle + self.max_angle/2) / self.max_angle * 255) & 0xFF,  # Angle normalized
            int(self.steering_torque) & 0xFF,                                      # Torque
            int(self.power_assist_level) & 0xFF,                                   # Assist level
            int(self.steering_temp) & 0xFF,                                        # Temperature
            int(self.vehicle_speed) & 0xFF,                                           # Vehicle speed
            0x00,                                                                      # Reserved
            0x00,                                                                      # Reserved
            0x00                                                                       # Checksum
        ]
        self.can_pub.publish(can_msg)
        
    def run(self):
        """Main node loop"""
        rate = rospy.Rate(50)  # 50Hz update rate
        
        while not rospy.is_shutdown():
            # Update steering angle
            self.update_steering_angle()
            
            # Calculate steering torque
            self.calculate_steering_torque()
            
            # Calculate steering temperature
            self.calculate_steering_temperature()
            
            # Publish status
            self.steering_angle_pub.publish(Float32(self.steering_angle))
            self.steering_torque_pub.publish(Float32(self.steering_torque))
            self.steering_assist_pub.publish(Float32(self.power_assist_level))
            self.steering_temp_pub.publish(Float32(self.steering_temp))
            
            # Send periodic CAN status
            if rospy.Time.now().to_sec() % 1.0 < 0.02:  # Every second
                self.send_steering_status()
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SteeringSystemNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
