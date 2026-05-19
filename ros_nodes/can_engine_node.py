#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from can_msgs.msg import Frame


class CANEngineNodeV2:
    def __init__(self):
        rospy.init_node('can_engine_v2', anonymous=True)

        self.can_sub = rospy.Subscriber('/can_rx', Frame, self.can_callback)

        self.rpm_pub = rospy.Publisher('/engine/rpm', Float32, queue_size=10)
        self.temp_pub = rospy.Publisher('/engine/temperature', Float32, queue_size=10)
        self.throttle_pos_pub = rospy.Publisher('/engine/throttle_position', Float32, queue_size=10)
        self.fuel_rate_pub = rospy.Publisher('/engine/fuel_rate', Float32, queue_size=10)

        self.can_pub = rospy.Publisher('/can_tx', Frame, queue_size=10)

        self.current_rpm = 800.0
        self.engine_temp = 90.0
        self.throttle_position = 0.0
        self.fuel_rate = 0.0
        self.last_status_time = rospy.Time(0)

        rospy.loginfo('CAN Engine Node V2 initialized')

    def can_callback(self, msg):
        if msg.id == 0x100 and len(msg.data) >= 1:
            self.process_throttle_command(int(msg.data[0]))

    def process_throttle_command(self, throttle_byte):
        throttle_byte = max(0, min(255, int(throttle_byte)))
        throttle = throttle_byte / 255.0

        self.throttle_position = throttle
        self.calculate_engine_response(throttle)
        self.publish_engine_status()
        self.send_engine_status_can()

    def calculate_engine_response(self, throttle):
        target_rpm = 800.0 + (throttle * 6000.0)
        rpm_diff = target_rpm - self.current_rpm
        self.current_rpm += rpm_diff * 0.1
        self.current_rpm = max(800.0, min(8000.0, self.current_rpm))

        self.fuel_rate = max(0.0, min(50.0, throttle * 50.0))

        target_temp = 90.0 + (throttle * 30.0)
        temp_diff = target_temp - self.engine_temp
        self.engine_temp += temp_diff * 0.01
        self.engine_temp = max(0.0, min(120.0, self.engine_temp))

    def publish_engine_status(self):
        self.rpm_pub.publish(Float32(self.current_rpm))
        self.temp_pub.publish(Float32(self.engine_temp))
        self.throttle_pos_pub.publish(Float32(self.throttle_position))
        self.fuel_rate_pub.publish(Float32(self.fuel_rate))

    def send_engine_status_can(self):
        rpm_raw = max(0, min(65535, int(round(self.current_rpm * 5.0))))
        temp_raw = max(0, min(255, int(round((self.engine_temp / 120.0) * 255.0))))
        fuel_raw = max(0, min(255, int(round((self.fuel_rate / 50.0) * 255.0))))

        can_msg = Frame()
        can_msg.id = 0x200
        can_msg.dlc = 8
        can_msg.data = [
            rpm_raw & 0xFF,
            (rpm_raw >> 8) & 0xFF,
            temp_raw,
            fuel_raw,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
        self.can_pub.publish(can_msg)
        self.last_status_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.publish_engine_status()
            if rospy.Time.now() - self.last_status_time > rospy.Duration(1.0):
                self.send_engine_status_can()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = CANEngineNodeV2()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('CAN Engine Node V2 shutting down')
