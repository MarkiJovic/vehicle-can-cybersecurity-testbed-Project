#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float32, Int32, Bool
from can_msgs.msg import Frame

class CarMonitorNode:
    def __init__(self):
        rospy.init_node('car_monitor', anonymous=True)
        
        # Subscribe to all car systems
        self.rpm_sub = rospy.Subscriber('/engine/rpm', Float32, self.rpm_callback)
        self.gear_sub = rospy.Subscriber('/transmission/gear', Int32, self.gear_callback)
        self.speed_sub = rospy.Subscriber('/vehicle/speed', Float32, self.speed_callback)
        self.brake_sub = rospy.Subscriber('/brake/pressure', Float32, self.brake_callback)
        self.steering_sub = rospy.Subscriber('/steering/angle', Float32, self.steering_callback)
        
        # CAN publisher for diagnostics
        self.can_pub = rospy.Publisher('/can_tx', Frame, queue_size=10)
        
        # System state
        self.engine_rpm = 800.0
        self.current_gear = 1
        self.vehicle_speed = 0.0
        self.brake_pressure = 0.0
        self.steering_angle = 0.0
        
        rospy.loginfo("Car Monitor Node initialized")
        
    def rpm_callback(self, msg):
        self.engine_rpm = msg.data
        
    def gear_callback(self, msg):
        self.current_gear = msg.data
        
    def speed_callback(self, msg):
        self.vehicle_speed = msg.data
        
    def brake_callback(self, msg):
        self.brake_pressure = msg.data
        
    def steering_callback(self, msg):
        self.steering_angle = msg.data
        
    def run(self):
        """Main monitoring loop"""
        rate = rospy.Rate(1)  # 1Hz update rate
        
        while not rospy.is_shutdown():
            # Print system status
            rospy.loginfo("=== VEHICLE STATUS ===")
            rospy.loginfo(f"Engine RPM: {self.engine_rpm:.0f}")
            rospy.loginfo(f"Gear: {self.current_gear}")
            rospy.loginfo(f"Speed: {self.vehicle_speed:.1f} km/h")
            rospy.loginfo(f"Brake Pressure: {self.brake_pressure:.1f} bar")
            rospy.loginfo(f"Steering Angle: {self.steering_angle:.1f}°")
            rospy.loginfo("======================")
            
            # Send diagnostic CAN message
            self.send_diagnostics()
            
            rate.sleep()
            
    def send_diagnostics(self):
        """Send diagnostic status via CAN"""
        can_msg = Frame()
        can_msg.id = 0x207  # Diagnostic message ID
        can_msg.dlc = 8
        can_msg.data = [
            int(self.engine_rpm / 64) & 0xFF,          # RPM high byte
            int(self.engine_rpm) & 0xFF,                # RPM low byte
            self.current_gear & 0xFF,                     # Current gear
            int(self.vehicle_speed) & 0xFF,               # Vehicle speed
            int(self.brake_pressure) & 0xFF,             # Brake pressure
            int((self.steering_angle + 270) / 540 * 255) & 0xFF,  # Steering normalized
            0x00,                                        # Reserved
            0x00                                         # Checksum
        ]
        self.can_pub.publish(can_msg)

if __name__ == '__main__':
    try:
        node = CarMonitorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
