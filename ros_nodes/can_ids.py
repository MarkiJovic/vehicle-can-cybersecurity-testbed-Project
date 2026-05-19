#!/usr/bin/env python3

import json
import time

import rospy
from can_msgs.msg import Frame
from std_msgs.msg import String


class IDS:
    def __init__(self):
        rospy.init_node('simple_can_ids', anonymous=True)

        self.pub = rospy.Publisher('/ids_alert', String, queue_size=20)

        rospy.Subscriber('/can_tx', Frame, self.tx_callback, queue_size=200)
        rospy.Subscriber('/can_rx', Frame, self.rx_callback, queue_size=200)

        self.baseline = {0x100: 0, 0x101: 127, 0x102: 0}
        self.last_alert_time = {}
        self.cooldown_sec = float(rospy.get_param('~cooldown_sec', 2.0))
        self.tx_times = {0x100: [], 0x101: [], 0x102: []}
        self.high_rate_threshold = float(rospy.get_param('~high_rate_threshold', 180.0))

        rospy.loginfo("Simple CAN IDS started")

    def should_emit(self, key):
        now = time.time()
        last = self.last_alert_time.get(key, 0.0)
        if now - last >= self.cooldown_sec:
            self.last_alert_time[key] = now
            return True
        return False

    def emit(self, rule, severity, details):
        payload = {
            "timestamp": time.time(),
            "rule": rule,
            "severity": severity,
            "details": details,
        }
        self.pub.publish(String(data=json.dumps(payload)))

    def rx_callback(self, msg):
        frame_id = int(msg.id)
        if frame_id in self.baseline and len(msg.data) > 0:
            self.baseline[frame_id] = int(msg.data[0]) & 0xFF

    def tx_callback(self, msg):
        frame_id = int(msg.id)
        if frame_id not in self.baseline or len(msg.data) == 0:
            return

        val = int(msg.data[0]) & 0xFF
        base = self.baseline.get(frame_id, 0)

        key = f"attack_tx_{frame_id}"
        if self.should_emit(key):
            self.emit("attack_tx_detected", "high", {
                "id": hex(frame_id),
                "value": val
            })

        now = time.time()
        times = self.tx_times[frame_id]
        times.append(now)
        self.tx_times[frame_id] = [t for t in times if now - t <= 1.0]
        rate = len(self.tx_times[frame_id])
        if rate > self.high_rate_threshold and self.should_emit(f"dos_{frame_id}"):
            self.emit("dos_attack", "high", {
                "id": hex(frame_id),
                "rate_hz": rate
            })

        if frame_id == 0x100 and val >= 240 and self.should_emit("throttle_attack"):
            self.emit("throttle_attack", "medium", {
                "id": hex(frame_id),
                "value": val
            })

        elif frame_id == 0x102 and val >= 240 and self.should_emit("brake_attack"):
            self.emit("brake_attack", "medium", {
                "id": hex(frame_id),
                "value": val
            })

        elif frame_id == 0x101 and abs(val - base) > 70 and self.should_emit("steering_attack"):
            self.emit("steering_attack", "high", {
                "id": hex(frame_id),
                "value": val,
                "baseline": base,
                "jump": abs(val - base)
            })


if __name__ == '__main__':
    IDS()
    rospy.spin()
