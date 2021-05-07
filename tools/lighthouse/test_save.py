from cflib.localization import LighthouseConfigFileManager
from datetime import datetime

now = datetime.now()

filename = now.strftime("%d-%m-%Y--%H:%M:%S") + '.json'

# retrieve geometries data from file.
geos = (LighthouseConfigFileManager.read('calibration_data.json'))[0]

# retrieve calibration data from file. (only here for the test_save.py not needed for get_bs_geometry)
calibs = (LighthouseConfigFileManager.read('calibration_data.json'))[1]

# retrieve system_type data from file.
system_type = (LighthouseConfigFileManager.read('calibration_data.json'))[2]

print("Configfile created", filename)

(LighthouseConfigFileManager.write(filename, geos=geos, calibs=calibs, system_type=system_type))
#
# with open(filename, 'w') as f:
#     #     f.write('\n')
#     f.write(LighthouseConfigFileManager.write(filename, geos=geos, calibs=calibs, system_type=system_type))