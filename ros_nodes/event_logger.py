#!/usr/bin/env python3

import csv
import json
import os
import time

import rospy
from can_msgs.msg import Frame
from std_msgs.msg import String


class Logger:
    def __init__(self):
        rospy.init_node('logger', anonymous=True)

        base_dir = os.path.expanduser("~/logs")
        session = time.strftime("session_%Y%m%d_%H%M%S")
        self.dir = os.path.join(base_dir, session)
        os.makedirs(self.dir, exist_ok=True)

        self.events_file = open(os.path.join(self.dir, "events.jsonl"), "a", encoding="utf-8")
        self.frames_file = open(os.path.join(self.dir, "control_frames.csv"), "a", newline="", encoding="utf-8")
        self.frames_writer = csv.writer(self.frames_file)
        self.frames_writer.writerow(["time", "topic", "frame_id", "dlc", "data"])

        rospy.Subscriber('/attack_status', String, self.log_event, callback_args="attack_status", queue_size=100)
        rospy.Subscriber('/ids_alert', String, self.log_event, callback_args="ids_alert", queue_size=100)
        rospy.Subscriber('/can_rx', Frame, self.log_frame, callback_args="/can_rx", queue_size=200)
        rospy.Subscriber('/can_tx', Frame, self.log_frame, callback_args="/can_tx", queue_size=200)

        rospy.loginfo("Logger writing to %s", self.dir)

    def log_event(self, msg, topic_name):
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"raw": msg.data}

        row = {
            "time": time.time(),
            "topic": topic_name,
            "payload": payload
        }
        self.events_file.write(json.dumps(row) + "\n")
        self.events_file.flush()

    def log_frame(self, msg, topic_name):
        frame_id = int(msg.id)
        if frame_id not in (0x100, 0x101, 0x102):
            return

        dlc = int(msg.dlc)
        data = list(msg.data[:dlc]) if dlc else list(msg.data)
        while len(data) < 8:
            data.append(0)

        self.frames_writer.writerow([time.time(), topic_name, hex(frame_id), dlc, data[:8]])
        self.frames_file.flush()

    def shutdown(self):
        try:
            self.events_file.close()
        except Exception:
            pass
        try:
            self.frames_file.close()
        except Exception:
            pass


if __name__ == '__main__':
    node = None
    try:
        node = Logger()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.shutdown()
