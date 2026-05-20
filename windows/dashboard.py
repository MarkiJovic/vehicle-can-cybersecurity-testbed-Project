#!/usr/bin/env python3
import json
import time
import tkinter as tk
from tkinter import ttk, messagebox
from collections import deque
import roslibpy


class Dashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("CAN Security Dashboard")
        self.root.geometry("1280x780")

        self.ros = None
        self.connected = False
        self.connection_status = tk.StringVar(value="Disconnected")

        self.can_rx_topic = None
        self.can_tx_topic = None
        self.attack_topic = None
        self.ids_topic = None
        self.sensor_attack_topic = None
        self.sensor_status_topic = None

        self.attack_value = tk.IntVar(value=255)
        self.sensor_distance_value = tk.DoubleVar(value=3.5)

        self.can_messages = deque(maxlen=250)
        self.security_alerts = []

        self.build_ui()
        self.root.after(300, self.update_gui)

    def build_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)

        top_frame = ttk.Frame(main_frame)
        top_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        top_frame.columnconfigure(4, weight=1)

        ttk.Label(top_frame, text="ROS Host").grid(row=0, column=0, sticky=tk.W)
        self.host_entry = ttk.Entry(top_frame, width=18)
        self.host_entry.insert(0, "192.168.56.104")
        self.host_entry.grid(row=0, column=1, padx=5)

        ttk.Button(top_frame, text="Connect", command=self.connect_to_ros).grid(row=0, column=2, padx=5)
        ttk.Label(top_frame, textvariable=self.connection_status).grid(row=0, column=3, padx=10)

        middle_frame = ttk.Frame(main_frame)
        middle_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        middle_frame.columnconfigure(0, weight=1)
        middle_frame.columnconfigure(1, weight=1)
        middle_frame.rowconfigure(0, weight=1)

        can_frame = ttk.LabelFrame(middle_frame, text="CAN Traffic", padding="10")
        can_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        can_frame.rowconfigure(0, weight=1)
        can_frame.columnconfigure(0, weight=1)

        self.can_text = tk.Text(can_frame, height=16, width=72)
        self.can_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        alert_frame = ttk.LabelFrame(middle_frame, text="Security Alerts", padding="10")
        alert_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        alert_frame.rowconfigure(0, weight=1)
        alert_frame.columnconfigure(0, weight=1)

        self.alert_text = tk.Text(alert_frame, height=16, width=58)
        self.alert_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        attack_frame = ttk.LabelFrame(main_frame, text="Attack Controls", padding="10")
        attack_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(10, 0))
        attack_frame.columnconfigure(1, weight=1)

        ttk.Label(attack_frame, text="Attack value").grid(row=0, column=0, sticky=tk.W)
        ttk.Scale(attack_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                  variable=self.attack_value, length=260).grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5)
        ttk.Label(attack_frame, textvariable=self.attack_value).grid(row=0, column=2, padx=5)

        ttk.Button(attack_frame, text="THROTTLE", command=lambda: self.launch_attack("throttle")).grid(row=0, column=3, padx=3)
        ttk.Button(attack_frame, text="STEERING", command=lambda: self.launch_attack("steering")).grid(row=0, column=4, padx=3)
        ttk.Button(attack_frame, text="BRAKE", command=lambda: self.launch_attack("brake")).grid(row=0, column=5, padx=3)
        ttk.Button(attack_frame, text="DOS", command=lambda: self.launch_attack("dos")).grid(row=0, column=6, padx=3)
        ttk.Button(attack_frame, text="REPLAY RECORD (5s)", command=self.replay_record).grid(row=0, column=7, padx=3)
        ttk.Button(attack_frame, text="REPLAY PLAY", command=self.replay_play).grid(row=0, column=8, padx=3)

        ttk.Separator(attack_frame, orient=tk.HORIZONTAL).grid(row=1, column=0, columnspan=9, sticky=(tk.W, tk.E), pady=10)

        ttk.Label(attack_frame, text="Fake obstacle distance (m)").grid(row=2, column=0, sticky=tk.W)
        ttk.Scale(attack_frame, from_=2.0, to=8.0, orient=tk.HORIZONTAL,
                  variable=self.sensor_distance_value, length=260).grid(row=2, column=1, sticky=(tk.W, tk.E), padx=5)
        ttk.Label(attack_frame, textvariable=self.sensor_distance_value).grid(row=2, column=2, padx=5)
        ttk.Button(attack_frame, text="SENSOR FAKE OBSTACLE ON",
                   command=self.enable_sensor_fake_obstacle).grid(row=2, column=3, columnspan=2, padx=3)
        ttk.Button(attack_frame, text="SENSOR ATTACK OFF",
                   command=self.disable_sensor_attack).grid(row=2, column=5, columnspan=2, padx=3)


    def connect_to_ros(self):
        host = self.host_entry.get().strip()
        port = 9090

        try:
            self.disconnect_from_ros()
            self.ros = roslibpy.Ros(host=host, port=port)
            self.ros.run()
            if not self.ros.is_connected:
                raise RuntimeError("Failed to connect")

            self.connected = True
            self.connection_status.set("Connected")
            self.subscribe_to_ros_topics()
            self.add_alert("Connected to ROS")
        except Exception as exc:
            self.connected = False
            self.connection_status.set("Disconnected")
            messagebox.showerror("ROS Connection Error", str(exc))

    def disconnect_from_ros(self):
        for topic in (self.can_rx_topic, self.can_tx_topic, self.attack_topic,
                      self.ids_topic, self.sensor_attack_topic, self.sensor_status_topic):
            try:
                if topic is not None:
                    topic.unsubscribe()
            except Exception:
                pass
        try:
            if self.ros is not None:
                self.ros.terminate()
        except Exception:
            pass

        self.ros = None
        self.connected = False
        self.connection_status.set("Disconnected")

    def subscribe_to_ros_topics(self):
        self.can_rx_topic = roslibpy.Topic(self.ros, '/can_rx', 'can_msgs/Frame')
        self.can_tx_topic = roslibpy.Topic(self.ros, '/can_tx', 'can_msgs/Frame')
        self.attack_topic = roslibpy.Topic(self.ros, '/attack_command', 'std_msgs/String')
        self.ids_topic = roslibpy.Topic(self.ros, '/ids_alert', 'std_msgs/String')
        self.sensor_attack_topic = roslibpy.Topic(self.ros, '/sensor_attack_command', 'std_msgs/String')
        self.sensor_status_topic = roslibpy.Topic(self.ros, '/sensor_attack_status', 'std_msgs/String')

        self.can_rx_topic.subscribe(self.can_rx_callback)
        self.can_tx_topic.subscribe(self.can_tx_callback)
        self.ids_topic.subscribe(self.ids_callback)
        self.sensor_status_topic.subscribe(self.sensor_status_callback)

        self.attack_topic.advertise()
        self.sensor_attack_topic.advertise()

    def normalize_can_bytes(self, data, dlc=0):
        import base64

        parsed = []

        if isinstance(data, str):
            s = data.strip()
            if s:
                try:
                    decoded = base64.b64decode(s, validate=True)
                    parsed = [b & 0xFF for b in decoded]
                except Exception:
                    parsed = [ord(ch) & 0xFF for ch in s]
        else:
            for value in list(data or []):
                try:
                    if isinstance(value, int):
                        parsed.append(value & 0xFF)
                    elif isinstance(value, str):
                        s = value.strip()
                        if not s:
                            continue
                        try:
                            decoded = base64.b64decode(s, validate=True)
                            if decoded:
                                parsed.extend([b & 0xFF for b in decoded])
                                continue
                        except Exception:
                            pass

                        if s.lower().startswith('0x'):
                            parsed.append(int(s, 16) & 0xFF)
                        else:
                            try:
                                parsed.append(int(s) & 0xFF)
                            except ValueError:
                                if len(s) == 1:
                                    parsed.append(ord(s) & 0xFF)
                                else:
                                    parsed.extend(ord(ch) & 0xFF for ch in s)
                    else:
                        parsed.append(int(value) & 0xFF)
                except Exception:
                    pass

        return parsed[:dlc] if dlc else parsed

    def frame_to_str(self, message, direction):
        frame_id = int(message.get('id', 0))
        dlc = int(message.get('dlc', 0) or 0)
        data = self.normalize_can_bytes(message.get('data', []) or [], dlc)
        return "%s %s %s" % (direction, hex(frame_id), data)

    def can_rx_callback(self, message):
        frame_id = int(message.get('id', 0))
        if 0x100 <= frame_id <= 0x10F:
            self.can_messages.appendleft(self.frame_to_str(message, 'RX'))

    def can_tx_callback(self, message):
        frame_id = int(message.get('id', 0))
        if 0x100 <= frame_id <= 0x10F:
            self.can_messages.appendleft(self.frame_to_str(message, 'TX'))

    def add_alert(self, text):
        self.security_alerts.append({'timestamp': time.time(), 'message': text})
        if len(self.security_alerts) > 120:
            self.security_alerts.pop(0)

    def launch_attack(self, attack_type):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to ROS first")
            return

        if attack_type == 'dos':
            payload = {'type': 'dos', 'duration': 6.0, 'message_count': 2000}
        else:
            payload = {'type': attack_type, 'value': int(self.attack_value.get()), 'duration': 5.0,
                       'message_count': 1200}

        try:
            msg = roslibpy.Message({'data': json.dumps(payload)})
            self.attack_topic.publish(msg)
            self.add_alert("Attack launched: %s" % attack_type)
        except Exception as exc:
            self.add_alert("Attack publish failed: %s" % exc)

    def replay_record(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to ROS first")
            return
        payload = {'type': 'replay_record', 'duration': 5.0}
        try:
            msg = roslibpy.Message({'data': json.dumps(payload)})
            self.attack_topic.publish(msg)
            self.add_alert("Replay recording started for 5 seconds")
        except Exception as exc:
            self.add_alert("Replay record failed: %s" % exc)

    def replay_play(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to ROS first")
            return
        payload = {'type': 'replay_play', 'repeat_count': 1}
        try:
            msg = roslibpy.Message({'data': json.dumps(payload)})
            self.attack_topic.publish(msg)
            self.add_alert("Replay play triggered")
        except Exception as exc:
            self.add_alert("Replay play failed: %s" % exc)

    def enable_sensor_fake_obstacle(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to ROS first")
            return

        payload = {
            'type': 'fake_obstacle',
            'enabled': True,
            'distance_m': round(float(self.sensor_distance_value.get()), 2),
            'width_m': 0.8,
            'points': 120,
            'timestamp': time.time()
        }

        try:
            msg = roslibpy.Message({'data': json.dumps(payload)})
            self.sensor_attack_topic.publish(msg)
            self.add_alert("Sensor attack ON: fake obstacle at %.2f m" % payload['distance_m'])
        except Exception as exc:
            self.add_alert("Sensor attack publish failed: %s" % exc)

    def disable_sensor_attack(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to ROS first")
            return

        payload = {'type': 'off', 'enabled': False, 'timestamp': time.time()}

        try:
            msg = roslibpy.Message({'data': json.dumps(payload)})
            self.sensor_attack_topic.publish(msg)
            self.add_alert("Sensor attack OFF")
        except Exception as exc:
            self.add_alert("Sensor attack disable failed: %s" % exc)

    def sensor_status_callback(self, message):
        try:
            raw = message.get('data', '{}')
            payload = json.loads(raw)
            state = payload.get('state', 'unknown')
            attack = payload.get('attack', 'unknown')
            text = "[SENSOR] %s | attack=%s" % (str(state).upper(), attack)

            if 'distance_m' in payload:
                text += " | distance=%sm" % payload['distance_m']
            if 'error' in payload:
                text += " | error=%s" % payload['error']
            if 'message' in payload:
                text += " | %s" % payload['message']
        except Exception as exc:
            text = "Sensor status parse error: %s" % exc

        self.add_alert(text)

    def ids_callback(self, message):
        try:
            raw = message.get('data', '{}')
            payload = json.loads(raw)
            rule = payload.get('rule', 'unknown')
            severity = payload.get('severity', 'info')
            details = payload.get('details', {})
            text = "[%s] %s | %s" % (str(severity).upper(), rule, details)
        except Exception as exc:
            text = "IDS parse error: %s" % exc
        self.add_alert(text)

    def update_gui(self):
        self.can_text.delete('1.0', tk.END)
        self.can_text.insert(tk.END, '\n'.join(list(self.can_messages)[:70]))

        self.alert_text.delete('1.0', tk.END)
        lines = []
        for item in self.security_alerts[-70:]:
            ts = time.strftime('%H:%M:%S', time.localtime(item['timestamp']))
            lines.append("[%s] %s" % (ts, item['message']))
        self.alert_text.insert(tk.END, '\n'.join(lines))

        self.root.after(300, self.update_gui)


if __name__ == '__main__':
    root = tk.Tk()
    style = ttk.Style()
    try:
        style.theme_use('clam')
    except Exception:
        pass
    Dashboard(root)
    root.mainloop()
