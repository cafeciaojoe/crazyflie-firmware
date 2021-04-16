#!/usr/bin/env python3

#  ,---------,       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Crazyflie control firmware
#
#  Copyright (C) 2020 - 2021 Bitcraze AB
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, in version 3.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>.
#
#
#  Calculate Lighthouse base station geometry based on data from the
#  Crayzyflie. Requires a lighthouse deck.
#
#  This script connects to the Crazyflie and reads the sweep angles
#  for the base station(s) and calculates their position and orientation in
#  a coordinate system with origin at the position of the Crazyflie.
#
#  Usage:
#  1. Place the Crazyflie in the origin of your coordinate system, facing
#     positive X.
#  2. Make sure the Lighthouse deck is mounted completely parallell with the
#     ground (Crazyflie PCB) since this is what is going to define the
#     coordiate system.
#  3. Run the script
#  4. Copy/paste the output into lighthouse.c, recompile and flash the Crazyflie.
#

import math
import time
import argparse
import logging
import numpy as np
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseBsGeometry
from cflib.crazyflie.mem import LighthouseMemHelper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.localization import LighthouseBsGeoEstimator
from cflib.localization import LighthouseSweepAngleAverageReader

from combine_measurements import combine_measurements


def bs_is_on_ceiling(bs_id):
    # When sideways
    if bs_id == 0:
        return True
    if bs_id == 1:
        return True
    if bs_id == 2:
        return False
    if bs_id == 3:
        return False


def yes_or_no(question):
    check = str(input(f"{question} (Y/N): ")).lower().strip()
    try:
        if check[0] == 'y':
            return True
        elif check[0] == 'n':
            return False
        else:
            print("Please enter a valid value (Y/N)")
            return yes_or_no(question)
    except Exception as error:
        print("Please enter a valid value (Y/N)")
        print(error)
        return yes_or_no(question)


def save_sensor_data(sensor_vectors_all):
    """
    Save raw data that CF is receiving from base stations
    """
    import pickle
    import datetime
    timestamp = str(datetime.datetime.now().timestamp()).replace(".", "")
    pickle.dump(
        sensor_vectors_all, open(f"sensor_data_{timestamp}.pickle", "wb")
    )


class Estimator:
    def __init__(self):
        self.sensor_vectors_all = None
        self.collection_event = Event()
        self.write_event = Event()

    def angles_collected_cb(self, angles):
        self.sensor_vectors_all = angles
        self.collection_event.set()

    def write_done_cb(self, success):
        if not success:
            print("Write to CF failed!")
        self.write_event.set()

    def estimate(self, uri, do_write):
        cf = Crazyflie(rw_cache='./cache')
        all_measurements = []

        with SyncCrazyflie(uri, cf=cf) as scf:

            user_wants_to_continue = True

            while user_wants_to_continue:

                time.sleep(5)

                print("Starting measurement procedure... ")
                cf_is_sideways = yes_or_no(
                    "Is the CF oriented on its side? "
                    "Note that the CF should not be on "
                    "its side for the first measurement."
                )

                if cf_is_sideways:
                    print(
                        "The CF is oriented on its side. Beginning sweep... "
                    )

                if not cf_is_sideways:
                    print(
                        "The CF is not oriented on its side. "
                        "Beginning sweep... "
                    )

                print("Reading sensor data...")
                sweep_angle_reader = LighthouseSweepAngleAverageReader(
                    scf.cf, self.angles_collected_cb
                )
                sweep_angle_reader.start_angle_collection()
                self.collection_event.wait()

                print("Estimating position of base stations...")
                geometries = {}
                lbge = LighthouseBsGeoEstimator()

                # Temporary: outputting sensor data for testing and debugging
                save_sensor_data(self.sensor_vectors_all)

                for bs_id in sorted(self.sensor_vectors_all.keys()):
                    print('bs id is',bs_id)
                    average_data = self.sensor_vectors_all[bs_id]
                    sensor_data = average_data[1]

                    rotation_bs_matrix, position_bs_vector = lbge.estimate_geometry(
                        sensor_data
                    )

                    # Adjust result if CF is sideways
                    if cf_is_sideways:
                        print(
                            "Adjusting BS estimation results for sideways "
                            "CF..."
                        )

                        if bs_is_on_ceiling(bs_id):
                            print(f"Adjusting BS {bs_id}...")
                            position_bs_vector[1] = -position_bs_vector[1]
                            position_bs_vector[0] = -position_bs_vector[0]

                            r_rot_y = np.array([
                                [math.cos(math.pi / 2), 0,
                                 math.sin(math.pi / 2)],
                                [0, 1, 0],
                                [-math.sin(math.pi / 2), 0,
                                 math.cos(math.pi / 2)]
                            ])

                            rotation_bs_matrix = np.dot(
                                rotation_bs_matrix,
                                r_rot_y
                            )

                    is_valid = lbge.sanity_check_result(position_bs_vector)

                    if is_valid:
                        geo = LighthouseBsGeometry()
                        geo.rotation_matrix = rotation_bs_matrix
                        geo.origin = position_bs_vector
                        geo.valid = True

                        geometries[bs_id] = geo

                        self.print_geo(rotation_bs_matrix, position_bs_vector,
                                       is_valid)
                    else:
                        print(f"Warning: could not find valid solution for "
                              f"{bs_id}")

                    all_measurements.append(geometries)

                print()
                user_wants_to_continue = yes_or_no(
                    "Would you like to take another measurement?")
                if user_wants_to_continue:
                    print("Sure! You may now move the drone.")

            print("All measurements taken. Combining measurements...")
            geometries = combine_measurements(all_measurements)

            if do_write:
                print("Uploading geo data to CF")
                helper = LighthouseMemHelper(scf.cf)
                helper.write_geos(geometries, self.write_done_cb)
                self.write_event.wait()

    def print_geo(self, rotation_cf, position_cf, is_valid):
        print('C-format')
        if is_valid:
            valid_c = 'true'
        else:
            valid_c = 'false'

        print('{.valid = ' + valid_c + ', .origin = {', end='')
        for i in position_cf:
            print("{:0.6f}, ".format(i), end='')

        print("}, .mat = {", end='')

        for i in rotation_cf:
            print("{", end='')
            for j in i:
                print("{:0.6f}, ".format(j), end='')
            print("}, ", end='')

        print("}},")

        print()
        print('python-format')
        print('geo = LighthouseBsGeometry()')
        print('geo.origin =', np.array2string(position_cf, separator=','))
        print('geo.rotation_matrix = [', end='')
        for row in rotation_cf:
            print(np.array2string(row, separator=','), end='')
            print(', ', end='')
        print(']')
        print('geo.valid =', is_valid)


parser = argparse.ArgumentParser()
uri = "radio://0/80/2M/A0A0A0A0A3"
parser.add_argument(
    "--uri",
    help="uri to use when connecting to the Crazyflie. Default: " + uri
)
parser.add_argument(
    "--write",
    help="upload the calculated geo data to the Crazflie",
    action="store_true"
)
args = parser.parse_args()
if args.uri:
    uri = args.uri

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers(enable_debug_driver=False)

estimator = Estimator()
estimator.estimate(uri, args.write)
