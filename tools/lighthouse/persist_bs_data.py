#!/usr/bin/env python3

#  ,---------,       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Crazyflie control firmware
#
#  Copyright (C) 2020 Bitcraze AB
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
#  Persist geometry and calibration data in the Crazyflie storage.
#
#  This script uploads geometry and calibration data to a crazyflie and
#  writes the data to persistant memory to make it available after
#  re-boot.
#
#  This script is a temporary solution until there is support
#  in the client.


import logging
import time

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseBsCalibration
from cflib.crazyflie.mem import LighthouseBsGeometry
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

uri = "radio://0/80/2M"


class WriteMem:
    def __init__(self, uri, geos, calibs):
        self.data_written = False
        self.result_received = False

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            mems = scf.cf.mem.get_mems(MemoryElement.TYPE_LH)

            count = len(mems)
            if count != 1:
                raise Exception('Unexpected nr of memories found:', count)

            lh_mem = mems[0]

            for bs, geo in geos.items():
                self.data_written = False
                print('Write geoetry', bs, 'to RAM')
                lh_mem.write_geo_data(bs, geo, self._data_written, write_failed_cb=self._data_failed)

                while not self.data_written:
                    time.sleep(0.1)

            for bs, calib in calibs.items():
                self.data_written = False
                print('Write calibration', bs, 'to RAM')
                lh_mem.write_calib_data(bs, calib, self._data_written, write_failed_cb=self._data_failed)

                while not self.data_written:
                    time.sleep(0.1)

            print('Persist data')
            scf.cf.loc.receivedLocationPacket.add_callback(self._data_persisted)
            scf.cf.loc.send_lh_persist_data_packet(list(range(16)), list(range(16)))

            while not self.result_received:
                time.sleep(0.1)


    def _data_written(self, mem, addr):
        self.data_written = True

    def _data_failed(self, mem, addr):
        raise Exception('Write to RAM failed')

    def _data_persisted(self, data):
        if (data.data):
            print('Data persisted')
        else:
            raise Exception("Write to storage failed")

        self.result_received = True


geo0 = LighthouseBsGeometry()
geo0.origin = [-0.57477044,-0.83214664, 2.48939668]
geo0.rotation_matrix = [[ 0.59742972,-0.65980477, 0.45577999], [0.53718765,0.75127166,0.38343228], [-0.59540504, 0.01576554, 0.80327099], ]
geo0.valid = True

geo1 = LighthouseBsGeometry()
geo1.origin = [1.49820781,0.73088941,2.40678524]
geo1.rotation_matrix = [[-0.5372352 , 0.74389358,-0.397493  ], [-0.49024846,-0.65890823,-0.57052291], [-0.68631974,-0.11163466, 0.71868138], ]
geo1.valid = True

# Lh2 - 0
calib0 = LighthouseBsCalibration()
calib0.sweeps[0].tilt = -0.051078
calib0.sweeps[0].phase = 0.000000
calib0.sweeps[0].curve = 0.247043
calib0.sweeps[0].gibphase = 1.665625
calib0.sweeps[0].gibmag = -0.011288
calib0.sweeps[0].ogeephase = 1.538609
calib0.sweeps[0].ogeemag = 0.471610
calib0.sweeps[1].tilt = 0.043630
calib0.sweeps[1].phase = -0.004223
calib0.sweeps[1].curve = 0.507986
calib0.sweeps[1].gibphase = 2.046868
calib0.sweeps[1].gibmag = -0.009459
calib0.sweeps[1].ogeephase = 2.389616
calib0.sweeps[1].ogeemag = 0.375476
calib0.valid = True

# LH2 = 1
calib1 = LighthouseBsCalibration()
calib1.sweeps[0].tilt = -0.049504
calib1.sweeps[0].phase = 0.000000
calib1.sweeps[0].curve = 0.069301
calib1.sweeps[0].gibphase = 1.765391
calib1.sweeps[0].gibmag = -0.024516
calib1.sweeps[0].ogeephase = 1.742652
calib1.sweeps[0].ogeemag = 1.290244
calib1.sweeps[1].tilt = 0.042626
calib1.sweeps[1].phase = -0.004367
calib1.sweeps[1].curve = 0.540287
calib1.sweeps[1].gibphase = 2.484687
calib1.sweeps[1].gibmag = -0.019553
calib1.sweeps[1].ogeephase = 2.698655
calib1.sweeps[1].ogeemag = 1.096106
calib1.valid = True

logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers(enable_debug_driver=False)

WriteMem(uri,
    {
        0: geo0,
        1: geo1,
    },
    {
        0: calib0,
        1: calib1,
    })
