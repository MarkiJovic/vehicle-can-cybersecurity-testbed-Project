#!/usr/bin/env python3

import rospy
import time
import math
from std_msgs.msg import Float32, Int32
from can_msgs.msg import Frame

class VehicleSpeedNode:
    def __init__(self):
        rospy.init_node('vehicle_speed', anonymous=True)
        
        # Subscribe to CAN bus
        self.can_sub = rospy.Subscriber('/can_rx', Frame, self.can_callback)
        
        # Subscribe to engine RPM
        self.rpm_sub = rospy.Subscriber('/engine/rpm', Float32, self.rpm_callback)
        
        # Subscribe to transmission gear
        self.gear_sub = rospy.Subscriber('/transmission/gear', Int32, self.gear_callback)
        
        # Publishers
        self.speed_pub = rospy.Publisher('/vehicle/speed', Float32, queue_size=10)
        self.wheel_speeds_pub = rospy.Publisher('/vehicle/wheel_speeds', Float32, queue_size=10)
        
        # CAN publisher
        self.can_pub = rospy.Publisher('/can_tx', Frame, queue_size=10)
        
        # Vehicle state
        self.engine_rpm = 800.0
        self.current_gear = 1
        self.vehicle_speed = 0.0  # km/h
        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
        
        # Vehicle parameters
        self.tire_radius = 0.3  # meters
        self.final_drive = 3.42
        self.gear_ratios = [3.67, 2.10, 1.36, 1.03, 0.84, 0.68]
        
        rospy.loginfo("Vehicle Speed Node initialized")
        
    def can_callback(self, msg):
        """Process incoming CAN messages"""
        # Process wheel speed sensors (ID 0x107)
        if msg.id == 0x107 and len(msg.data) >= 4:
            # Extract individual wheel speeds
            fl_speed = (msg.data[0] << 8 | msg.data[1]) / 100.0  # Front Left
            fr_speed = (msg.data[2] << 8 | msg.data[3]) / 100.0  # Front Right
            
            self.wheel_speeds[0] = fl_speed
            self.wheel_speeds[1] = fr_speed
            
            # Calculate vehicle speed from wheel speeds
            avg_wheel_speed = (fl_speed + fr_speed) / 2.0
            self.vehicle_speed = avg_wheel_speed * 3.6  # Convert m/s to km/h
            
        # Process rear wheel speeds (ID 0x108)
        elif msg.id == 0x108 and len(msg.data) >= 4:
            rl_speed = (msg.data[0] << 8 | msg.data[1]) / 100.0  # Rear Left
            rr_speed = (msg.data[2] << 8 | msg.data[3]) / 100.0  # Rear Right
            
            self.wheel_speeds[2] = rl_speed
            self.wheel_speeds[3] = rr_speed
            
    def rpm_callback(self, msg):
        """Process engine RPM updates"""
        self.engine_rpm = msg.data
        self.calculate_speed_from_drivetrain()
        
    def gear_callback(self, msg):
        """Process transmission gear updates"""
        self.current_gear = msg.data
        self.calculate_speed_from_drivetrain()
        
    def calculate_speed_from_drivetrain(self):
        """Calculate vehicle speed from engine RPM and gear"""
        if self.current_gear <= 0 or self.current_gear > 6:
            return
            
        # Get current gear ratio
        if self.current_gear <= 6:
            gear_ratio = self.gear_ratios[self.current_gear - 1]
        else:
            gear_ratio = 1.0
            
        # Calculate wheel speed (m/s)
        # Engine RPM -> Wheel RPM -> Linear speed
        wheel_rpm = self.engine_rpm / (gear_ratio * self.final_drive)
        wheel_speed_mps = wheel_rpm * 2 * math.pi * self.tire_radius / 60.0
        
        # Convert to km/h
        calculated_speed = wheel_speed_mps * 3.6
        
        # Smooth speed calculation
        self.vehicle_speed = self.vehicle_speed * 0.8 + calculated_speed * 0.2
        
    def send_speed_status(self):
        """Send vehicle speed status via CAN"""
        can_msg = Frame()
        can_msg.id = 0x206
        can_msg.dlc = 8
        can_msg.data = [
            int(self.vehicle_speed) & 0xFF,                    # Vehicle speed
            int(self.engine_rpm / 10) & 0xFF,                  # Engine RPM (divided)
            self.current_gear & 0xFF,                            # Current gear
            0x00,                                                # Reserved
            0x00,                                                # Reserved
            0x00,                                                # Reserved
            0x00,                                                # Reserved
            0x00                                                 # Checksum
        ]
        self.can_pub.publish(can_msg)
        
    def run(self):
        """Main node loop"""
        rate = rospy.Rate(50)  # 50Hz update rate
        
        while not rospy.is_shutdown():
            # Publish vehicle speed
            self.speed_pub.publish(Float32(self.vehicle_speed))
            
            # Publish average wheel speed
            avg_wheel_speed = sum(self.wheel_speeds) / 4.0
            self.wheel_speeds_pub.publish(Float32(avg_wheel_speed))
            
            # Send periodic CAN status
            if rospy.Time.now().to_sec() % 1.0 < 0.02:  # Every second
                self.send_speed_status()
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = VehicleSpeedNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
