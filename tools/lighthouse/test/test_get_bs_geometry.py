import pickle

from cflib.localization import LighthouseBsGeoEstimator
from cflib.crazyflie.mem import LighthouseBsGeometry


def test_get_bs_geometry_upways():
    upways_data = "sensor_data_1614655937013395.pickle"
    bs_poses = get_bs_geometry_from_sensor_data(upways_data)
    print(bs_poses)


def test_get_bs_geometry_sideways():
    sideways_data = "sensor_data_1614655377286432.pickle"
    bs_poses = get_bs_geometry_from_sensor_data(sideways_data, sideways=True)
    print(bs_poses)


def get_bs_geometry_from_sensor_data(sensor_data_path, sideways=False):
    sensor_data = pickle.load(open(sensor_data_path, "rb"))
    bs_poses = run_get_bs_geometry_proxy(sensor_data, sideways)
    return bs_poses


def run_get_bs_geometry_proxy(sensor_vectors_all, sideways=False):

    estimator = LighthouseBsGeoEstimator()
    geometries = {}

    for id in sorted(sensor_vectors_all.keys()):
        average_data = sensor_vectors_all[id]
        sensor_data = average_data[1]
        rotation_bs_matrix, position_bs_vector = estimator.estimate_geometry(
            sensor_data, id, sideways)
        is_valid = estimator.sanity_check_result(position_bs_vector)
        if is_valid:
            geo = LighthouseBsGeometry()
            geo.rotation_matrix = rotation_bs_matrix
            geo.origin = position_bs_vector
            geo.valid = True

            geometries[id] = geo

            # self.print_geo(rotation_bs_matrix, position_bs_vector, is_valid)

        else:
            print(f"Warning: could not find valid solution for {id}")

    return geometries