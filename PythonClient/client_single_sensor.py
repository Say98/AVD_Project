#!/usr/bin/env python3

from __future__ import print_function

import argparse
import logging
import random
import time
import cv2
import numpy as np
import pygame

#   Required to import carla library
import os
import sys
sys.path.append(os.path.abspath(sys.path[0] + '/..'))


# Carla Imports
from carla.client import make_carla_client
from carla.sensor import Camera, Lidar
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line
import carla.image_converter as image_converter


def make_settings(args,sensor_select,synchronous_mode):
    if args.settings_filepath is None:
        # Create a CarlaSettings object.
        settings = CarlaSettings()
        settings.set(
            SynchronousMode=synchronous_mode,
            SendNonPlayerAgentsInfo=False,
            NumberOfVehicles=0,
            NumberOfPedestrians=0,
            WeatherId=random.choice([1, 3, 7, 8, 14]),
            QualityLevel=args.quality_level)
        settings.randomize_seeds()

        # The default camera captures RGB images of the scene.
        # Sensor name identifies the instance.
        camera0 = Camera('CameraRGB')

        # set pixel Resolution: WIDTH * HEIGHT
        camera0.set_image_size(args.window_width, args.window_height)

        # set position X (front), Y (lateral), Z (height) relative to the car in meters
        # (0,0,0) is at center of baseline of car 
        camera0.set_position(0.30, 0, 1.30)

        # Adding camera to configuration 
        settings.add_sensor(camera0)

    else:
        # Load the ClientSettings.ini
        with open(args.settings_filepath, 'r') as fp:
            settings = fp.read()
    
    return settings


def run_carla_client(args):
    # Here we will run 3 episodes with 300 frames each.
    number_of_episodes = 4
    frames_per_episode = 100
    synchronous_mode = False

    # Create the connection with the server already connected at args.host : args.port
    with make_carla_client(args.host, args.port, timeout = None) as client:
        print('CarlaClient connected')

        for episode in range(0, number_of_episodes):
            # Start a new episode : 
            #   Each episode has an own setup
            #   A single connection can manage more  episodes
            settings = make_settings(args,episode,synchronous_mode)
            
            # Loading settings into the server.
            # Scene object with map_name, startins_spots and sensors configured.
            scene = client.load_settings(settings)
            
            # Visualize the possible starting position and choose one
            # print("Starting Position : ",scene.player_start_spots)
            player_start = random.randint(0, max(0, len(scene.player_start_spots) - 1))

            # Starting the episode at player_start index
            print('Starting new episode at %r...' % scene.map_name)
            client.start_episode(player_start)

            # Iterate every frame in the episode.
            for frame in range(0, frames_per_episode):

                # Read the data produced by the server this frame.
                measurements, sensor_data = client.read_data()
                
                # Print some of the measurements.
                print_measurements(measurements)

                # Visualize data from sensors
                visualize_sensor_data(sensor_data,episode)    

                if synchronous_mode:
                    # forward step if sync mode
                    client.send_control(throttle=0.1,
                                            brake=0.0,
                                            steer=0.0,
                                            hand_brake=False,
                                            reverse=False)
            
            cv2.destroyAllWindows()
                 

def visualize_sensor_data(sensor_data,sensor_select,showing_dims=(200,200)):
    if sensor_data.get("CameraRGB",None) is not None:
        # Camera RGB data
        image_RGB = image_converter.to_rgb_array(sensor_data["CameraRGB"])
        image_RGB = cv2.resize(image_RGB,showing_dims)
        cv2.imshow("RGB_IMAGE",image_RGB)
        cv2.waitKey(1)

        image_BGR = image_converter.to_bgra_array(sensor_data["CameraRGB"])
        image_BGR = cv2.resize(image_BGR,showing_dims)
        cv2.imshow("BGRA_IMAGE",image_BGR)
        cv2.waitKey(1)


def print_measurements(measurements):
    number_of_agents = len(measurements.non_player_agents)
    player_measurements = measurements.player_measurements
    message = 'Vehicle at ({pos_x:.1f}, {pos_y:.1f}), '
    message += '{speed:.0f} km/h, '
    message += 'Collision: {{vehicles={col_cars:.0f}, pedestrians={col_ped:.0f}, other={col_other:.0f}}}, '
    message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road, '
    message += '({agents_num:d} non-player agents in the scene)'
    message = message.format(
        pos_x=player_measurements.transform.location.x,
        pos_y=player_measurements.transform.location.y,
        speed=player_measurements.forward_speed * 3.6, # m/s -> km/h
        col_cars=player_measurements.collision_vehicles,
        col_ped=player_measurements.collision_pedestrians,
        col_other=player_measurements.collision_other,
        other_lane=100 * player_measurements.intersection_otherlane,
        offroad=100 * player_measurements.intersection_offroad,
        agents_num=number_of_agents)
    print_over_same_line(message)


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '-l', '--lidar',
        action='store_true',
        help='enable Lidar')
    argparser.add_argument(
        '-ww', '--window_width',
        default=200,
        type=int,
        help='window width')
    argparser.add_argument(
        '-wh', '--window_height',
        default=200,
        type=int,
        help='window height')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Low',
        help='graphics quality level, a lower level makes the simulation run considerably faster.')
    argparser.add_argument(
        '-i', '--images-to-disk',
        action='store_true',
        dest='save_images_to_disk',
        help='save images (and Lidar data if active) to disk')
    argparser.add_argument(
        '-c', '--carla-settings',
        metavar='PATH',
        dest='settings_filepath',
        default=None,
        help='Path to a "CarlaSettings.ini" file')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    while True:
        try:
            run_carla_client(args)

            print('Done.')
            return

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
