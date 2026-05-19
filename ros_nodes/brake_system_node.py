#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float32, Bool
from can_msgs.msg import Frame


class BrakeSystemNode:
    def __init__(self):
        rospy.init_node('brake_system', anonymous=True)

        self.can_sub = rospy.Subscriber('/can_rx', Frame, self.can_callback)
        self.speed_sub = rospy.Subscriber('/vehicle/speed', Float32, self.speed_callback)

        self.brake_pressure_pub = rospy.Publisher('/brake/pressure', Float32, queue_size=10)
        self.abs_status_pub = rospy.Publisher('/brake/abs_status', Bool, queue_size=10)
        self.brake_temp_pub = rospy.Publisher('/brake/temperature', Float32, queue_size=10)
        self.wheel_speed_pub = rospy.Publisher('/brake/wheel_speeds', Float32, queue_size=10)

        self.can_pub = rospy.Publisher('/can_tx', Frame, queue_size=10)

        self.brake_pressure = 0.0
        self.target_pressure = 0.0
        self.vehicle_speed = 0.0
        self.brake_temp = 90.0
        self.abs_active = False
        self.abs_timer = rospy.Time(0)

        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.max_pressure = 100.0
        self.abs_threshold = 0.3
        self.brake_bias = 0.6
        self.brake_pad_wear = 20.0
        self.brake_fluid_level = 220.0
        self.last_status_time = rospy.Time(0)

        rospy.loginfo('Brake System Node initialized')

    def can_callback(self, msg):
        if msg.id == 0x102 and len(msg.data) >= 1:
            self.process_brake_command(int(msg.data[0]))
        elif msg.id == 0x105 and len(msg.data) >= 1 and int(msg.data[0]) == 0x01:
            self.abs_active = False
            rospy.logwarn('ABS disabled by command')

    def speed_callback(self, msg):
        self.vehicle_speed = max(0.0, msg.data)
        self.calculate_wheel_speeds()
        self.check_abs_conditions()

    def process_brake_command(self, brake_byte):
        self.target_pressure = (max(0, min(255, brake_byte)) / 255.0) * self.max_pressure

    def calculate_wheel_speeds(self):
        base_speed = self.vehicle_speed * 1000.0 / 3600.0
        if self.brake_pressure > 0:
            slip_factor = min(self.brake_pressure / self.max_pressure, 1.0) * 0.1
            self.wheel_speeds[0] = base_speed * (1 - slip_factor * self.brake_bias)
            self.wheel_speeds[1] = base_speed * (1 - slip_factor * self.brake_bias)
            self.wheel_speeds[2] = base_speed * (1 - slip_factor * (1 - self.brake_bias))
            self.wheel_speeds[3] = base_speed * (1 - slip_factor * (1 - self.brake_bias))
        else:
            self.wheel_speeds = [base_speed, base_speed, base_speed, base_speed]

    def check_abs_conditions(self):
        if self.vehicle_speed < 5.0:
            self.abs_active = False
            return
        avg_wheel_speed = sum(self.wheel_speeds) / 4.0 if self.wheel_speeds else 0.0
        for i, wheel_speed in enumerate(self.wheel_speeds):
            if avg_wheel_speed > 0:
                slip_ratio = (avg_wheel_speed - wheel_speed) / avg_wheel_speed
                if abs(slip_ratio) > self.abs_threshold:
                    if not self.abs_active:
                        rospy.logwarn(f'ABS activated - Wheel {i} slip: {slip_ratio:.3f}')
                        self.abs_active = True
                        self.abs_timer = rospy.Time.now()
                    self.apply_abs_modulation(i, slip_ratio)
                    return
        if self.abs_active and (rospy.Time.now() - self.abs_timer > rospy.Duration(2.0)):
            self.abs_active = False

    def apply_abs_modulation(self, wheel_index, slip_ratio):
        modulation_factor = 1.0 - min(abs(slip_ratio), 1.0)
        if wheel_index < 2:
            self.wheel_speeds[wheel_index] *= (0.8 + modulation_factor * 0.2)
        else:
            self.wheel_speeds[wheel_index] *= (0.9 + modulation_factor * 0.1)

    def calculate_brake_temperature(self):
        base_temp = 90.0
        braking_factor = (self.brake_pressure / self.max_pressure) ** 2
        speed_factor = min(self.vehicle_speed / 100.0, 1.0)
        target_temp = base_temp + (braking_factor * 200.0) + (speed_factor * 50.0)
        cooling_rate = 0.02 if self.brake_pressure < 10.0 else 0.005
        target_temp -= (self.brake_temp - base_temp) * cooling_rate
        temp_diff = target_temp - self.brake_temp
        self.brake_temp += temp_diff * 0.05
        self.brake_temp = max(0.0, min(400.0, self.brake_temp))

    def update_brake_pressure(self):
        pressure_diff = self.target_pressure - self.brake_pressure
        self.brake_pressure += pressure_diff * 0.1
        if self.abs_active:
            abs_modulation = math.sin(rospy.Time.now().to_sec() * 20.0) * 5.0
            self.brake_pressure += abs_modulation
        self.brake_pressure = max(0.0, min(self.max_pressure, self.brake_pressure))

    def send_brake_status(self):
        pressure_raw = max(0, min(255, int(round((self.brake_pressure / 100.0) * 255.0))))
        temp_raw = max(0, min(255, int(round((self.brake_temp / 400.0) * 255.0))))
        pad_wear_raw = max(0, min(255, int(round(self.brake_pad_wear))))
        fluid_level_raw = max(0, min(255, int(round(self.brake_fluid_level))))

        abs_bits = 0
        if self.abs_active:
            abs_bits |= 0x01

        can_msg = Frame()
        can_msg.id = 0x204
        can_msg.dlc = 8
        can_msg.data = [
            pressure_raw,
            abs_bits,
            temp_raw,
            pad_wear_raw,
            fluid_level_raw,
            0x00,
            0x00,
            0x00,
        ]
        self.can_pub.publish(can_msg)
        self.last_status_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.update_brake_pressure()
            self.calculate_brake_temperature()

            self.brake_pressure_pub.publish(Float32(self.brake_pressure))
            self.abs_status_pub.publish(Bool(self.abs_active))
            self.brake_temp_pub.publish(Float32(self.brake_temp))
            avg_wheel_speed = sum(self.wheel_speeds) / 4.0 if self.wheel_speeds else 0.0
            self.wheel_speed_pub.publish(Float32(avg_wheel_speed))

            if rospy.Time.now() - self.last_status_time > rospy.Duration(1.0):
                self.send_brake_status()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = BrakeSystemNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
