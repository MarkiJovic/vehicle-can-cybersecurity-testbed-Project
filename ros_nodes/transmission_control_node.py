#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32, Bool
from can_msgs.msg import Frame


class TransmissionControlNode:
    def __init__(self):
        rospy.init_node('transmission_control', anonymous=True)

        self.can_sub = rospy.Subscriber('/can_rx', Frame, self.can_callback)
        self.rpm_sub = rospy.Subscriber('/engine/rpm', Float32, self.rpm_callback)
        self.speed_sub = rospy.Subscriber('/vehicle/speed', Float32, self.speed_callback)

        self.gear_pub = rospy.Publisher('/transmission/gear', Int32, queue_size=10)
        self.trans_temp_pub = rospy.Publisher('/transmission/temperature', Float32, queue_size=10)
        self.clutch_status_pub = rospy.Publisher('/transmission/clutch_status', Bool, queue_size=10)
        self.shift_indicator_pub = rospy.Publisher('/transmission/shift_indicator', Bool, queue_size=10)

        self.can_pub = rospy.Publisher('/can_tx', Frame, queue_size=10)

        self.current_gear = 1
        self.target_gear = 1
        self.engine_rpm = 800.0
        self.vehicle_speed = 0.0
        self.transmission_temp = 90.0
        self.clutch_engaged = True
        self.is_shifting = False
        self.shift_timer = rospy.Time(0)

        self.upshift_points = [2500, 2800, 3000, 3200, 3500]
        self.downshift_points = [1500, 1800, 2000, 2200, 2500]

        rospy.loginfo('Transmission Control Node initialized')

    def can_callback(self, msg):
        if msg.id == 0x103 and len(msg.data) >= 1:
            self.process_shift_command(int(msg.data[0]))
        elif msg.id == 0x104 and len(msg.data) >= 1:
            self.process_mode_command(int(msg.data[0]))

    def rpm_callback(self, msg):
        self.engine_rpm = msg.data
        self.calculate_automatic_shifts()

    def speed_callback(self, msg):
        self.vehicle_speed = max(0.0, msg.data)

    def process_shift_command(self, shift_cmd):
        if shift_cmd == 0x01 and self.current_gear < 6:
            self.target_gear = self.current_gear + 1
            self.initiate_shift()
        elif shift_cmd == 0x02 and self.current_gear > 1:
            self.target_gear = self.current_gear - 1
            self.initiate_shift()
        elif shift_cmd == 0x03:
            self.target_gear = 0
            self.initiate_shift()
        elif shift_cmd == 0x04:
            self.target_gear = -1
            self.initiate_shift()

    def process_mode_command(self, mode_cmd):
        if mode_cmd == 0x01:
            self.upshift_points = [3000, 3300, 3500, 3700, 4000]
        elif mode_cmd == 0x02:
            self.upshift_points = [2200, 2500, 2700, 2900, 3200]
        else:
            self.upshift_points = [2500, 2800, 3000, 3200, 3500]

    def calculate_automatic_shifts(self):
        if self.is_shifting:
            return
        if 0 < self.current_gear <= 5 and self.engine_rpm > self.upshift_points[self.current_gear - 1]:
            self.target_gear = self.current_gear + 1
            self.initiate_shift()
        elif self.current_gear > 1 and self.engine_rpm < self.downshift_points[self.current_gear - 2]:
            self.target_gear = self.current_gear - 1
            self.initiate_shift()

    def initiate_shift(self):
        if self.target_gear == self.current_gear:
            return
        rospy.loginfo(f'Initiating shift: {self.current_gear} -> {self.target_gear}')
        self.is_shifting = True
        self.clutch_engaged = False
        self.shift_timer = rospy.Time.now()
        self.shift_indicator_pub.publish(Bool(True))
        shift_time = 0.3 + abs(self.target_gear - self.current_gear) * 0.1
        rospy.Timer(rospy.Duration(shift_time), self.complete_shift, oneshot=True)

    def complete_shift(self, _event):
        self.current_gear = self.target_gear
        self.is_shifting = False
        self.clutch_engaged = True
        self.gear_pub.publish(Int32(self.current_gear))
        self.clutch_status_pub.publish(Bool(True))
        self.shift_indicator_pub.publish(Bool(False))
        self.send_transmission_status()

    def calculate_transmission_temp(self):
        base_temp = 90.0
        rpm_factor = max(0.0, (self.engine_rpm - 800.0) / 6000.0)
        load_factor = min(abs(self.current_gear) / 6.0, 1.0)
        target_temp = base_temp + (rpm_factor * 30.0) + (load_factor * 20.0)
        temp_diff = target_temp - self.transmission_temp
        self.transmission_temp += temp_diff * 0.01
        self.transmission_temp = max(0.0, min(150.0, self.transmission_temp))

    def send_transmission_status(self):
        speed_raw = max(0, min(255, int(round((self.vehicle_speed / 200.0) * 255.0))))
        temp_raw = max(0, min(255, int(round((self.transmission_temp / 150.0) * 255.0))))

        if self.current_gear < 0:
            gear_raw = 255
        elif self.current_gear == 0:
            gear_raw = 0
        else:
            gear_raw = max(1, min(6, self.current_gear))

        status = 0
        if self.current_gear != 0:
            status |= 0x01
        if self.clutch_engaged:
            status |= 0x02
        if self.is_shifting:
            status |= 0x04

        can_msg = Frame()
        can_msg.id = 0x203
        can_msg.dlc = 8
        can_msg.data = [
            gear_raw & 0xFF,
            speed_raw,
            temp_raw,
            status,
            gear_raw & 0xFF,
            0x00,
            0x00,
            0x00,
        ]
        self.can_pub.publish(can_msg)
        self.shift_timer = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.calculate_transmission_temp()
            self.gear_pub.publish(Int32(self.current_gear))
            self.trans_temp_pub.publish(Float32(self.transmission_temp))
            self.clutch_status_pub.publish(Bool(self.clutch_engaged))
            if rospy.Time.now() - self.shift_timer > rospy.Duration(1.0):
                self.send_transmission_status()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = TransmissionControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
