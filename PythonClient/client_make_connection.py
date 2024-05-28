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
            NumberOfVehicles=10,
            NumberOfPedestrians=10,
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
    number_of_episodes = 1
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
            print('Starting new episode at %r' % scene.map_name)
            print('Sensors loaded %r' % scene.sensors)
                        
            client.start_episode(player_start)
                

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
