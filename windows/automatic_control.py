#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Extended version of automatic vehicle control with unlimited runtime."""

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref
import can
import threading
import time
import json
import base64

try:
    import roslibpy
except ImportError:
    roslibpy = None

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error


# ==============================================================================
# -- CAN-driven control state -------------------------------------------------
# ==============================================================================

latest_throttle = 0.0
latest_steering = 0.0
latest_brake = 0.0
last_control_update = 0.0
control_state_lock = threading.Lock()
ros_control = None
ros_can_rx_topic = None
ros_can_tx_topic = None
ros_attack_status_topic = None
attack_active = False
attack_end_time = 0.0


def set_latest_control(throttle=None, steering=None, brake=None):
    global latest_throttle, latest_steering, latest_brake, last_control_update
    with control_state_lock:
        if throttle is not None:
            latest_throttle = max(0.0, min(1.0, float(throttle)))
        if steering is not None:
            latest_steering = max(-1.0, min(1.0, float(steering)))
        if brake is not None:
            latest_brake = max(0.0, min(1.0, float(brake)))
        last_control_update = time.time()


def get_latest_control():
    with control_state_lock:
        return latest_throttle, latest_steering, latest_brake, last_control_update


def clear_latest_control():
    global latest_throttle, latest_steering, latest_brake, last_control_update
    with control_state_lock:
        latest_throttle = 0.0
        latest_steering = 0.0
        latest_brake = 0.0
        last_control_update = 0.0


def normalize_can_bytes(data, dlc=0):
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


def ros_can_rx_callback(message):
    try:
        frame_id = int(message.get('id', 0))
        dlc = int(message.get('dlc', 0))
        data = normalize_can_bytes(message.get('data', []) or [], dlc)

        if not data:
            return

        if frame_id == 0x100:
            set_latest_control(throttle=data[0] / 255.0)
            print(f"CONTROL RX 0x100 -> throttle={data[0]}")
        elif frame_id == 0x101:
            set_latest_control(steering=(data[0] / 127.5) - 1.0)
            print(f"CONTROL RX 0x101 -> steering={data[0]}")
        elif frame_id == 0x102:
            set_latest_control(brake=data[0] / 255.0)
            print(f"CONTROL RX 0x102 -> brake={data[0]}")
    except Exception as e:
        print(f"ROS CAN RX parse error: {e}")


def attack_status_callback(message):
    global attack_active, attack_end_time
    try:
        raw = message.get('data', '{}') if isinstance(message, dict) else '{}'
        payload = json.loads(raw)

        state = str(payload.get('state', '')).lower()
        if state == 'started':
            duration = float(payload.get('duration', 3.0))
            attack_active = True
            attack_end_time = time.time() + max(0.1, duration)
            print(f"Attack status: started ({payload.get('type', 'unknown')}) duration={duration}")
        elif state in ('finished', 'cancelled', 'error'):
            attack_active = False
            attack_end_time = 0.0
            clear_latest_control()
            if state == 'error':
                print(f"Attack status: error -> {payload.get('error', 'unknown error')}")
            else:
                print(f"Attack status: {state}")
    except Exception as e:
        print(f"Attack status parse error: {e}")


def start_ros_control_listener(rosbridge_host):
    global ros_control, ros_can_rx_topic, ros_can_tx_topic
    if roslibpy is None:
        print("roslibpy not available; ROS CAN listener disabled")
        return False

    try:
        ros_control = roslibpy.Ros(host=rosbridge_host, port=9090)
        ros_control.run()
        if not ros_control.is_connected:
            print("Failed to connect to rosbridge for CAN-driven control")
            return False

        global ros_attack_status_topic
        ros_can_rx_topic = roslibpy.Topic(ros_control, '/can_rx', 'can_msgs/Frame')
        ros_can_rx_topic.subscribe(ros_can_rx_callback)
        ros_can_tx_topic = roslibpy.Topic(ros_control, '/can_tx', 'can_msgs/Frame')
        ros_can_tx_topic.subscribe(ros_can_rx_callback)
        ros_attack_status_topic = roslibpy.Topic(ros_control, '/attack_status', 'std_msgs/String')
        ros_attack_status_topic.subscribe(attack_status_callback)
        print(f"ROS CAN listener started via {rosbridge_host}:9090")
        return True
    except Exception as e:
        print(f"Failed to start ROS CAN listener: {e}")
        return False


def stop_ros_control_listener():
    global ros_control, ros_can_rx_topic, ros_can_tx_topic, ros_attack_status_topic
    try:
        if ros_can_rx_topic is not None:
            ros_can_rx_topic.unsubscribe()
    except Exception:
        pass
    try:
        if ros_can_tx_topic is not None:
            ros_can_tx_topic.unsubscribe()
    except Exception:
        pass
    try:
        if ros_attack_status_topic is not None:
            ros_attack_status_topic.unsubscribe()
    except Exception:
        pass
    try:
        if ros_control is not None:
            ros_control.terminate()
    except Exception:
        pass
    ros_can_rx_topic = None
    ros_can_tx_topic = None
    ros_attack_status_topic = None
    ros_control = None


# ==============================================================================
# -- Extended CAN Bridge Class -----------------------------------------------
# ==============================================================================

class CANBridge:
    def __init__(self):
        try:
            self.bus = can.interface.Bus(
                channel='PCAN_USBBUS1',
                bustype='pcan',
                bitrate=500000
            )
            self.running = True
            self.receiver_thread = None
            print("CAN bridge initialized successfully")
        except Exception as e:
            print(f"CAN bridge failed: {e}")
            self.bus = None
            self.running = False
            self.receiver_thread = None

    def send_control_data(self, throttle, steering, brake):
        """Send vehicle control data over CAN"""
        if not self.bus or not self.running:
            return

        try:
            throttle_byte = int(max(0, min(throttle * 255, 255)))
            steering_byte = int(max(0, min((steering + 1.0) * 127.5, 255)))
            brake_byte = int(max(0, min(brake * 255, 255)))

            throttle_msg = can.Message(
                arbitration_id=0x100,
                data=[throttle_byte, 0, 0, 0, 0, 0, 0, 0],
                is_extended_id=False
            )
            self.bus.send(throttle_msg)

            steering_msg = can.Message(
                arbitration_id=0x101,
                data=[steering_byte, 0, 0, 0, 0, 0, 0, 0],
                is_extended_id=False
            )
            self.bus.send(steering_msg)

            brake_msg = can.Message(
                arbitration_id=0x102,
                data=[brake_byte, 0, 0, 0, 0, 0, 0, 0],
                is_extended_id=False
            )
            self.bus.send(brake_msg)

            print(f"CAN TX -> throttle={throttle_byte} steering={steering_byte} brake={brake_byte}")

        except Exception as e:
            print(f"CAN send error: {e}")

    def start_receiver(self):
        """Read control frames from CAN and use them as CARLA authority."""
        if not self.bus or not self.running or self.receiver_thread is not None:
            return

        def receive_loop():
            while self.running:
                try:
                    msg = self.bus.recv(timeout=0.05)
                    if msg is None:
                        continue

                    if msg.arbitration_id == 0x100 and len(msg.data) >= 1:
                        set_latest_control(throttle=msg.data[0] / 255.0)
                    elif msg.arbitration_id == 0x101 and len(msg.data) >= 1:
                        set_latest_control(steering=(msg.data[0] / 127.5) - 1.0)
                    elif msg.arbitration_id == 0x102 and len(msg.data) >= 1:
                        set_latest_control(brake=msg.data[0] / 255.0)
                except Exception as e:
                    print(f"CAN RX error: {e}")
                    time.sleep(0.1)

        self.receiver_thread = threading.Thread(target=receive_loop, daemon=True)
        self.receiver_thread.start()
        print("CAN receiver started")

    def cleanup(self):
        """Clean up CAN connection"""
        self.running = False
        if self.receiver_thread and self.receiver_thread.is_alive():
            self.receiver_thread.join(timeout=1.0)
        if self.bus:
            self.bus.shutdown()



def force_all_traffic_lights_green(world, freeze=True):
    """Set all traffic lights to green and optionally freeze them."""
    try:
        lights = world.get_actors().filter('traffic.traffic_light*')
        count = 0
        for actor in lights:
            actor.set_state(carla.TrafficLightState.Green)
            actor.freeze(bool(freeze))
            count += 1
        print("Traffic lights forced to green (%d lights, freeze=%s)" % (count, freeze))
    except Exception as e:
        print("Failed to force traffic lights green: %s" % e)


# Global CAN bridge instance
can_bridge = CANBridge()


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    """Method to find weather presets"""
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    """Method to get actor display name"""
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


def get_available_vehicles(world):
    """Get list of available vehicle blueprints"""
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    vehicles = []
    for blueprint in vehicle_blueprints:
        vehicles.append(blueprint.id)
    return sorted(vehicles)


def select_vehicle(vehicle_type, world):
    """Select and return a specific vehicle blueprint"""
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    
    # Try to find exact match
    for blueprint in vehicle_blueprints:
        if blueprint.id.lower() == vehicle_type.lower():
            return blueprint
    
    # Try partial match
    for blueprint in vehicle_blueprints:
        if vehicle_type.lower() in blueprint.id.lower():
            return blueprint
    
    print(f"Vehicle '{vehicle_type}' not found. Available vehicles:")
    for blueprint in vehicle_blueprints[:10]:  # Show first 10
        print(f"  - {blueprint.id}")
    
    return random.choice(vehicle_blueprints)  # Fallback to random


# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
    """ Class representing the surrounding environment """

    def __init__(self, carla_world, hud, args):
        """Constructor method"""
        self._args = args
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self.restart(args)
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.start_time = time.time()
        self.running_time = 0

    def restart(self, args):
        """Restart: world"""
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # Get vehicle blueprint
        if hasattr(args, 'vehicle_type') and args.vehicle_type:
            blueprint = select_vehicle(args.vehicle_type, self.world)
            print(f"Selected vehicle: {blueprint.id}")
        else:
            # Default to Audi E-Tron
            blueprint = select_vehicle("vehicle.audi.etron", self.world)
            print(f"Default vehicle selected: {blueprint.id}")
        
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)

        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)

        if self._args.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

        # Set up sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_id
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        
        # Initialize agent after player is spawned
        self.agent = None
        
        # Set up agent with arguments from command line
        self.set_agent(self._args.agent, self._args.behavior)
        
    def set_agent(self, agent_type="Behavior", behavior="normal"):
        """Initialize and set up agent"""
        if self.player is None:
            print("Cannot set agent: player not spawned")
            return
            
        if agent_type == "Basic":
            self.agent = BasicAgent(self.player)
        else:
            self.agent = BehaviorAgent(self.player, behavior=behavior)
        
        # Set initial destination
        spawn_points = self.map.get_spawn_points()
        if spawn_points:
            destination = random.choice(spawn_points).location
            self.agent.set_destination(destination)
            print(f"Agent destination set to: {destination}")
            self.hud.notification("Destination set", seconds=3.0)
        else:
            print("No spawn points available for destination setting")

    def next_weather(self, reverse=False):
        """Get next weather setting"""
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        """Method for every tick"""
        self.running_time = time.time() - self.start_time
        self.hud.tick(self, clock)

    def render(self, display):
        """Render world"""
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        """Destroy sensors"""
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        """Destroys all actors"""
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self, world):
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

    @staticmethod
    def _is_quit_shortcut(key):
        """Shortcut for quitting"""
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    """Class for HUD text"""

    def __init__(self, width, height):
        """Constructor method"""
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        """Gets informations from the world at every tick"""
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame_count
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        """HUD method for every tick"""
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        transform = world.player.get_transform()
        vel = world.player.get_velocity()
        control = world.player.get_control()
        heading = 'N' if abs(transform.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(transform.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > transform.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > transform.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Run Time: % 12s' % datetime.timedelta(seconds=int(world.running_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (transform.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (transform.location.x, transform.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % transform.location.z,
            '']

        if isinstance(control, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', control.throttle, 0.0, 1.0),
                ('Steer:', control.steer, -1.0, 1.0),
                ('Brake:', control.brake, 0.0, 1.0),
                ('Reverse:', control.reverse),
                ('Hand brake:', control.hand_brake),
                ('Manual:', control.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(control.gear, control.gear)]
        elif isinstance(control, carla.WalkerControl):
            self._info_text += [
                ('Speed:', control.speed, 0.0, 5.556),
                ('Jump:', control.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]

        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']

        def dist(l):
            return math.sqrt((l.x - transform.location.x)**2 + (l.y - transform.location.y)
                             ** 2 + (l.z - transform.location.z)**2)
        vehicles = [(dist(x.get_location()), x) for x in vehicles if x.id != world.player.id]

        for dist, vehicle in sorted(vehicles):
            if dist > 200.0:
                break
            vehicle_type = get_actor_display_name(vehicle, truncate=22)
            self._info_text.append('% 4dm %s' % (dist, vehicle_type))

    def toggle_info(self):
        """Toggle info on or off"""
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        """Notification text"""
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        """Error text"""
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        """Render for HUD class"""
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        fig = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect(
                                (bar_h_offset + fig * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (fig * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
            self._notifications.render(display)
            self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    """ Class for fading text """

    def __init__(self, font, dim, pos):
        """Constructor method"""
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        """Set fading text"""
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        """Fading text method for every tick"""
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        """Render fading text method"""
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """ Helper class for text render"""

    def __init__(self, font, width, height):
        """Constructor method"""
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for i, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, i * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        """Toggle on or off the render help"""
        self._render = not self._render

    def render(self, display):
        """Render help text method"""
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    """ Class for collision sensors"""

    def __init__(self, parent_actor, hud):
        """Constructor method"""
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        """Gets the history of collisions"""
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        """On collision method"""
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    """Class for lane invasion sensors"""

    def __init__(self, parent_actor, hud):
        """Constructor method"""
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        """On invasion method"""
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    """ Class for GNSS sensors"""

    def __init__(self, parent_actor):
        """Constructor method"""
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(carla.Location(x=1.0, z=2.8)),
                                        attach_to=self._parent)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        """GNSS method"""
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    """ Class for camera management"""

    def __init__(self, parent_actor, hud):
        """Constructor method"""
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(
                carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), attachment.SpringArm),
            (carla.Transform(
                carla.Location(x=1.6, z=1.7)), attachment.Rigid),
            (carla.Transform(
                carla.Location(x=5.5, y=1.5, z=1.5)), attachment.SpringArm),
            (carla.Transform(
                carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), attachment.SpringArm),
            (carla.Transform(
                carla.Location(x=-1, y=-bound_y, z=0.5)), attachment.Rigid)]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            blp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                blp.set_attribute('image_size_x', str(hud.dim[0]))
                blp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                blp.set_attribute('range', '50')
            item.append(blp)
        self.index = None

    def toggle_camera(self):
        """Activate a camera"""
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        """Set a sensor"""
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else (
            force_respawn or (self.sensors[index][0] != self.sensors[self.index][0]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to
            # self to avoid circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def toggle_recording(self):
        """Toggle recording on or off"""
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        """Render method"""
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=assignment-from-no-return
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- Extended Game Loop ---------------------------------------------------------
# ==============================================================================


def game_loop(args):
    """
    Extended main loop of the simulation with unlimited runtime.
    It handles updating all the HUD information, ticking the agent and, if needed, the world.
    """

    pygame.init()
    pygame.font.init()
    world = None

    try:
        if args.seed:
            random.seed(args.seed)

        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)

        traffic_manager = client.get_trafficmanager()
        sim_world = client.get_world()

        if args.sync:
            settings = sim_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager.set_synchronous_mode(True)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args)
        controller = KeyboardControl(world)

        if args.all_green_lights:
            force_all_traffic_lights_green(world.world, freeze=True)

        start_ros_control_listener(args.rosbridge_host)

        # Agent is now set up in World.restart()
        # Just pass the agent type and behavior to World
        
        clock = pygame.time.Clock()

        print("Starting extended CARLA simulation with unlimited runtime...")
        print("Press Ctrl+C or ESC to stop the simulation")
        print("Press 'H' for help")

        while True:
            clock.tick_busy_loop(30)
            if args.sync:
                world.world.tick()
            else:
                world.world.wait_for_tick()
            if controller.parse_events():
                break

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

            # Check if agent exists and reached destination
            if world.agent and world.agent.done():
                if args.loop:
                    # Set new destination
                    spawn_points = world.map.get_spawn_points()
                    if spawn_points:
                        destination = random.choice(spawn_points).location
                        world.agent.set_destination(destination)
                        world.hud.notification("New target set, continuing simulation...", seconds=3.0)
                        print("New target set, continuing simulation...")
                else:
                    print("Target reached, but continuing simulation (unlimited mode)")
                    # Don't break - continue running indefinitely

            # CAN-driven control:
            # agent computes desired control, sends it to CAN,
            # and CARLA applies the latest control values decoded from ROS /can_rx.
            if world.agent:
                desired_control = world.agent.run_step()
                desired_control.manual_gear_shift = False

                active_attack_now = attack_active and (time.time() < attack_end_time)

                can_throttle, can_steering, can_brake, can_time = get_latest_control()

                decoded_control = carla.VehicleControl()

                if active_attack_now and (time.time() - can_time < 0.2):
                    print("ATTACK ACTIVE -> CARLA using CAN control")
                    decoded_control.throttle = can_throttle
                    decoded_control.steer = can_steering
                    decoded_control.brake = can_brake
                else:
                    # Normal driving -> agent only (stable lane following)
                    decoded_control.throttle = desired_control.throttle
                    decoded_control.steer = desired_control.steer
                    decoded_control.brake = desired_control.brake

                    # Still mirror agent control to CAN for monitoring / replay capture
                    if can_bridge.running:
                        can_bridge.send_control_data(
                            desired_control.throttle,
                            desired_control.steer,
                            desired_control.brake
                        )

                decoded_control.hand_brake = False
                decoded_control.manual_gear_shift = False

                world.player.apply_control(decoded_control)

    finally:

        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(True)

            world.destroy()


        stop_ros_control_listener()

        # Clean up CAN bridge
        if can_bridge.running:
            can_bridge.cleanup()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================


def main():
    """Main method"""

    argparser = argparse.ArgumentParser(
        description='CARLA Extended Automatic Control Client (Unlimited Runtime)')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution (recommended for stable timing)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='Actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--vehicle',
        metavar='VEHICLE_TYPE',
        dest='vehicle_type',
        help='Select specific vehicle (e.g., "vehicle.audi.etron", "vehicle.tesla.model3")')
    argparser.add_argument(
        '--list-vehicles',
        action='store_true',
        help='List all available vehicles and exit')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching previous one (continuous mode)')
    argparser.add_argument(
        "-a", "--agent", type=str,
        choices=["Behavior", "Basic"],
        help="select which agent to run",
        default="Behavior")
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None) ',
        default=None,
        type=int)
    argparser.add_argument(
        '--timeout',
        type=int,
        default=0,  # 0 means unlimited
        help='Simulation timeout in seconds (0 = unlimited, default: unlimited)')
    argparser.add_argument(
        '--rosbridge-host',
        dest='rosbridge_host',
        default='192.168.56.104',
        help='ROS bridge host for /can_rx feedback (default: 192.168.56.104)')
    argparser.add_argument(
        '--all-green-lights',
        action='store_true',
        dest='all_green_lights',
        help='Force all traffic lights to green and freeze them for demos')
    args = argparser.parse_args()

    # Handle list vehicles option
    if args.list_vehicles:
        try:
            client = carla.Client(args.host, args.port)
            client.set_timeout(4.0)
            world = client.get_world()
            vehicles = get_available_vehicles(world)
            print("Available vehicles:")
            for i, vehicle in enumerate(vehicles, 1):
                print(f"  {i:3d}. {vehicle}")
            print(f"\nTotal vehicles: {len(vehicles)}")
            print("\nUsage examples:")
            print("  python automatic_control_extended.py --vehicle vehicle.audi.etron")
            print("  python automatic_control_extended.py --vehicle vehicle.tesla.model3")
            print("  python automatic_control_extended.py --vehicle vehicle.bmw.isetta")
        except Exception as e:
            print(f"Error listing vehicles: {e}")
        return

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as e:
        print(f'\nError: {e}')


if __name__ == '__main__':
    main()
