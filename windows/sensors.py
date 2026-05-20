#!/usr/bin/env python3
from __future__ import print_function

import argparse
import glob
import json
import math
import os
import sys
import time
import weakref

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import numpy as np
import pygame
import roslibpy


class LidarSafetyApp:
    def __init__(self, client, args):
        self.client = client
        self.world = client.get_world()
        self.args = args

        self.hero = None
        self.camera = None
        self.lidar = None

        self.display = None
        self.clock = None
        self.font = None
        self.camera_surface = None

        self.latest_camera_frame = -1
        self.latest_lidar_frame = -1
        self.latest_lidar_points = 0

        self.nearest_front_distance = None
        self.front_point_count = 0
        self.safety_state = "NORMAL"

        self.sensor_attack_active = False
        self.sensor_attack_name = "OFF"

        self.fake_obstacle_distance = 3.5
        self.fake_obstacle_width = 0.8
        self.fake_obstacle_points = 120

        self.ros = None
        self.sensor_attack_topic = None
        self.sensor_status_topic = None

    def find_hero(self):
        vehicles = self.world.get_actors().filter('vehicle.*')
        for actor in vehicles:
            if actor.attributes.get('role_name', '') == 'hero':
                return actor
        return None

    def wait_for_hero(self, timeout=30.0):
        start = time.time()
        while time.time() - start < timeout:
            hero = self.find_hero()
            if hero is not None:
                return hero
            print("Waiting for hero vehicle...")
            time.sleep(1.0)
        raise RuntimeError("Hero vehicle not found. Start your main driving script first.")

    def setup_ui(self):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode((self.args.width, self.args.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("CARLA Hero Sensors + LiDAR Safety")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(pygame.font.get_default_font(), 18)

    def start_ros_listener(self):
        if not self.args.rosbridge_host:
            return

        try:
            self.ros = roslibpy.Ros(host=self.args.rosbridge_host, port=self.args.rosbridge_port)
            self.ros.run()

            if self.ros.is_connected:
                self.sensor_attack_topic = roslibpy.Topic(self.ros, '/sensor_attack_command', 'std_msgs/String')
                self.sensor_attack_topic.subscribe(self.sensor_attack_callback)

                self.sensor_status_topic = roslibpy.Topic(self.ros, '/sensor_attack_status', 'std_msgs/String')
                self.sensor_status_topic.advertise()

                self.publish_sensor_status('ready', {'message': 'LiDAR safety node connected'})
                print("Connected to rosbridge at %s:%s" % (self.args.rosbridge_host, self.args.rosbridge_port))
            else:
                print("Warning: failed to connect to rosbridge")
        except Exception as exc:
            print("Warning: rosbridge connection failed: %s" % exc)

    def stop_ros_listener(self):
        try:
            if self.sensor_attack_topic is not None:
                self.sensor_attack_topic.unsubscribe()
        except Exception:
            pass
        try:
            if self.sensor_status_topic is not None:
                self.sensor_status_topic.unadvertise()
        except Exception:
            pass
        try:
            if self.ros is not None:
                self.ros.terminate()
        except Exception:
            pass

    def publish_sensor_status(self, state, extra=None):
        if self.sensor_status_topic is None:
            return
        payload = {'state': state, 'timestamp': time.time(), 'attack': self.sensor_attack_name}
        if extra:
            payload.update(extra)
        try:
            self.sensor_status_topic.publish(roslibpy.Message({'data': json.dumps(payload)}))
        except Exception:
            pass

    def sensor_attack_callback(self, message):
        try:
            raw = message.get('data', '{}') if isinstance(message, dict) else '{}'
            payload = json.loads(raw)
        except Exception as exc:
            self.publish_sensor_status('error', {'error': 'Invalid sensor attack JSON: %s' % exc})
            return

        attack_type = str(payload.get('type', '')).strip().lower()
        enabled = bool(payload.get('enabled', True))

        if attack_type in ('off', 'disable', 'none') or not enabled:
            self.sensor_attack_active = False
            self.sensor_attack_name = 'OFF'
            self.publish_sensor_status('disabled', {'message': 'Sensor attack disabled'})
            return

        if attack_type == 'fake_obstacle':
            self.sensor_attack_active = True
            self.sensor_attack_name = 'FAKE_OBSTACLE'
            self.fake_obstacle_distance = float(payload.get('distance_m', 3.5))
            self.fake_obstacle_width = float(payload.get('width_m', 0.8))
            self.fake_obstacle_points = int(payload.get('points', 120))
            self.publish_sensor_status('enabled', {
                'distance_m': self.fake_obstacle_distance,
                'width_m': self.fake_obstacle_width,
                'points': self.fake_obstacle_points
            })
            return

        self.publish_sensor_status('error', {'error': 'Unknown sensor attack type: %s' % attack_type})

    def attach_sensors(self):
        bp_lib = self.world.get_blueprint_library()

        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self.args.width))
        camera_bp.set_attribute('image_size_y', str(self.args.height))
        camera_bp.set_attribute('fov', str(self.args.camera_fov))

        camera_transform = carla.Transform(carla.Location(x=1.6, z=1.7), carla.Rotation(pitch=0.0))
        self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.hero)

        lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(self.args.lidar_channels))
        lidar_bp.set_attribute('range', str(self.args.lidar_range))
        lidar_bp.set_attribute('rotation_frequency', str(self.args.lidar_rotation_hz))
        lidar_bp.set_attribute('points_per_second', str(self.args.lidar_points_per_second))
        lidar_bp.set_attribute('upper_fov', str(self.args.lidar_upper_fov))
        lidar_bp.set_attribute('lower_fov', str(self.args.lidar_lower_fov))

        if self.args.no_lidar_noise:
            lidar_bp.set_attribute('dropoff_general_rate', '0.0')
            lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
            lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
            lidar_bp.set_attribute('noise_stddev', '0.0')

        lidar_transform = carla.Transform(carla.Location(x=0.9, z=1.8), carla.Rotation())
        self.lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.hero)

        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: LidarSafetyApp._camera_callback(weak_self, image))
        self.lidar.listen(lambda cloud: LidarSafetyApp._lidar_callback(weak_self, cloud))

    @staticmethod
    def _camera_callback(weak_self, image):
        self = weak_self()
        if not self:
            return

        self.latest_camera_frame = image.frame
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3][:, :, ::-1]
        self.camera_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def _inject_fake_obstacle_points(self, front_points):
        if not self.sensor_attack_active or self.sensor_attack_name != 'FAKE_OBSTACLE':
            return front_points

        n = max(10, self.fake_obstacle_points)
        xs = np.random.normal(loc=self.fake_obstacle_distance, scale=0.20, size=n)
        ys = np.random.uniform(-self.fake_obstacle_width / 2.0, self.fake_obstacle_width / 2.0, size=n)
        zs = np.random.uniform(-0.8, 1.2, size=n)
        fake_points = np.stack([xs, ys, zs], axis=1).astype(np.float32)

        if front_points.size == 0:
            return fake_points
        return np.vstack([front_points, fake_points])

    @staticmethod
    def _lidar_callback(weak_self, cloud):
        self = weak_self()
        if not self:
            return

        self.latest_lidar_frame = cloud.frame
        self.latest_lidar_points = len(cloud)

        data = np.frombuffer(cloud.raw_data, dtype=np.float32)
        points = np.reshape(data, (-1, 4))[:, :3]

        front_x = points[:, 0]
        front_y = points[:, 1]
        front_z = points[:, 2]

        mask = (
            (front_x > self.args.front_min_x) &
            (front_x < self.args.front_max_x) &
            (np.abs(front_y) < self.args.front_half_width) &
            (front_z > self.args.front_min_z) &
            (front_z < self.args.front_max_z)
        )

        front_points = points[mask]
        front_points = self._inject_fake_obstacle_points(front_points)
        self.front_point_count = int(front_points.shape[0])

        if self.front_point_count == 0:
            self.nearest_front_distance = None
            return

        distances = np.linalg.norm(front_points[:, :2], axis=1)
        self.nearest_front_distance = float(np.min(distances))

    def apply_safety_override(self):
        if self.hero is None:
            return

        control = self.hero.get_control()
        new_control = carla.VehicleControl()

        new_control.throttle = control.throttle
        new_control.steer = control.steer
        new_control.brake = control.brake
        new_control.hand_brake = control.hand_brake
        new_control.reverse = control.reverse
        new_control.manual_gear_shift = control.manual_gear_shift
        new_control.gear = control.gear

        if self.nearest_front_distance is None:
            self.safety_state = "NORMAL"
            return

        if self.nearest_front_distance < self.args.brake_distance:
            new_control.throttle = 0.0
            new_control.brake = max(new_control.brake, self.args.brake_strength)
            self.safety_state = "BRAKE"
            self.hero.apply_control(new_control)
        elif self.nearest_front_distance < self.args.slow_distance:
            new_control.throttle = min(new_control.throttle, self.args.slow_throttle_cap)
            self.safety_state = "SLOW"
            self.hero.apply_control(new_control)
        else:
            self.safety_state = "NORMAL"

    def draw_overlay(self):
        hero_velocity = self.hero.get_velocity()
        speed_kmh = 3.6 * math.sqrt(hero_velocity.x ** 2 + hero_velocity.y ** 2 + hero_velocity.z ** 2)

        nearest_text = "None" if self.nearest_front_distance is None else "%.2f m" % self.nearest_front_distance
        lines = [
            "Hero: %s" % self.hero.type_id,
            "Speed: %.1f km/h" % speed_kmh,
            "Camera frame: %s" % self.latest_camera_frame,
            "LiDAR frame: %s" % self.latest_lidar_frame,
            "LiDAR points: %s" % self.latest_lidar_points,
            "Front-zone points: %s" % self.front_point_count,
            "Nearest front obstacle: %s" % nearest_text,
            "Safety state: %s" % self.safety_state,
            "Sensor attack: %s" % self.sensor_attack_name,
            "ESC / Q to quit",
        ]

        y = 8
        for line in lines:
            text_surface = self.font.render(line, True, (255, 255, 255))
            shadow = self.font.render(line, True, (0, 0, 0))
            self.display.blit(shadow, (9, y + 1))
            self.display.blit(text_surface, (8, y))
            y += 22

    def run(self):
        self.setup_ui()
        self.hero = self.wait_for_hero()
        print("Found hero vehicle: %s" % self.hero.type_id)
        self.attach_sensors()
        self.start_ros_listener()
        print("Attached front RGB camera and LiDAR to hero vehicle")
        print("LiDAR safety controller enabled")

        try:
            while True:
                self.clock.tick(30)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                    if event.type == pygame.KEYUP:
                        if event.key in (pygame.K_ESCAPE, pygame.K_q):
                            return

                self.apply_safety_override()

                if self.camera_surface is not None:
                    self.display.blit(self.camera_surface, (0, 0))
                else:
                    self.display.fill((20, 20, 20))

                self.draw_overlay()
                pygame.display.flip()

        finally:
            self.destroy()

    def destroy(self):
        self.stop_ros_listener()

        if self.camera is not None:
            try:
                self.camera.stop()
            except Exception:
                pass
            try:
                self.camera.destroy()
            except Exception:
                pass
            self.camera = None

        if self.lidar is not None:
            try:
                self.lidar.stop()
            except Exception:
                pass
            try:
                self.lidar.destroy()
            except Exception:
                pass
            self.lidar = None

        pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="Attach LiDAR to hero vehicle and apply simple safety override")
    parser.add_argument('--host', default='127.0.0.1', help='CARLA host (default: 127.0.0.1)')
    parser.add_argument('--port', '-p', default=2000, type=int, help='CARLA port (default: 2000)')
    parser.add_argument('--res', default='1280x720', help='Window resolution WIDTHxHEIGHT (default: 1280x720)')
    parser.add_argument('--camera-fov', default=90.0, type=float, help='Front camera FOV (default: 90)')
    parser.add_argument('--rosbridge-host', default='192.168.56.104', help='Ubuntu rosbridge host')
    parser.add_argument('--rosbridge-port', default=9090, type=int, help='Ubuntu rosbridge port')
    parser.add_argument('--lidar-channels', default=32, type=int, help='LiDAR channels (default: 32)')
    parser.add_argument('--lidar-range', default=70.0, type=float, help='LiDAR range in meters (default: 70)')
    parser.add_argument('--lidar-rotation-hz', default=20.0, type=float, help='LiDAR rotation frequency (default: 20)')
    parser.add_argument('--lidar-points-per-second', default=100000, type=int, help='LiDAR points/sec (default: 100000)')
    parser.add_argument('--lidar-upper-fov', default=10.0, type=float, help='LiDAR upper FOV (default: 10)')
    parser.add_argument('--lidar-lower-fov', default=-30.0, type=float, help='LiDAR lower FOV (default: -30)')
    parser.add_argument('--no-lidar-noise', action='store_true', help='Disable LiDAR noise/dropoff')
    parser.add_argument('--front-min-x', default=0.5, type=float, help='Front zone min x (m)')
    parser.add_argument('--front-max-x', default=12.0, type=float, help='Front zone max x (m)')
    parser.add_argument('--front-half-width', default=1.8, type=float, help='Front zone half-width (m)')
    parser.add_argument('--front-min-z', default=-1.5, type=float, help='Front zone min z (m)')
    parser.add_argument('--front-max-z', default=2.5, type=float, help='Front zone max z (m)')
    parser.add_argument('--slow-distance', default=10.0, type=float, help='Distance to start slowing (m)')
    parser.add_argument('--brake-distance', default=5.0, type=float, help='Distance to brake (m)')
    parser.add_argument('--slow-throttle-cap', default=0.20, type=float, help='Throttle cap in slow mode')
    parser.add_argument('--brake-strength', default=0.85, type=float, help='Brake strength in brake mode')

    args = parser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    app = LidarSafetyApp(client, args)
    app.run()


if __name__ == '__main__':
    main()
