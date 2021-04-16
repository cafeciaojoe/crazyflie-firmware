import math
import numpy as np
from cflib.crazyflie.mem import LighthouseBsGeometry
from scipy.spatial.transform import Rotation
from collections import namedtuple

System = namedtuple("System", ['P', 'R', 'bs'])


def gen_rot_x(angle):
    co = math.cos(angle)
    si = math.sin(angle)

    return np.array([
        [1, 0, 0],
        [0, co, -si],
        [0, si, co]])


def gen_rot_z(angle):
    co = math.cos(angle)
    si = math.sin(angle)

    return np.array([
        [co, -si, 0],
        [si, co, 0],
        [0, 0, 1]])


# Transform a point in ref frame x to ref frame r using transformation x->r
def transform_point_x_to_r(v_x, Px_r, Rx_r):
    return np.dot(Rx_r, v_x) + Px_r


# Transform a rotation in ref frame x to ref frame r using transformation x->r
def transform_rot_x_to_r(Rx_y, Ry_r):
    return np.dot(Ry_r, Rx_y)


# Transform a base station (y) in ref frame x to ref frame r using transformation x->r
def transform_x_to_r(Sy_x, Sx_r):
    Py_r = transform_point_x_to_r(Sy_x.P, Sx_r.P, Sx_r.R)
    Ry_r = transform_rot_x_to_r(Sy_x.R, Sx_r.R)
    return System(Py_r, Ry_r, Sy_x.bs)


# Transform a point in ref frame r to ref frame x using transformation x->r
def transform_point_to_x_from_r(v_r, Px_r, Rx_r):
    Rr_x = np.matrix.transpose(Rx_r)
    return np.dot(Rr_x, v_r - Px_r)


# Find the transformation x->r when we know the bs pos/rot for both x and r
def transform_from_ref_x_to_r_same_bs(Sb_x, Sb_r):
    Rb_x_inv = np.matrix.transpose(Sb_x.R)
    Rx_r = np.dot(Sb_r.R, Rb_x_inv)
    Px_r = Sb_r.P - np.dot(Rx_r, Sb_x.P)

    return System(Px_r, Rx_r, Sb_x.bs)


# Averaging of quaternions
# From https://stackoverflow.com/a/61013769
def q_average(Q, W=None):
    if W is not None:
        Q *= W[:, None]
    eigvals, eigvecs = np.linalg.eig(Q.T@Q)
    return eigvecs[:, eigvals.argmax()]


def system_average(S):
    bs = S[0].bs
    for s in S:
        if s.bs != bs:
            raise Exception("Different base stations")

    Q = map(lambda s : Rotation.from_matrix(s.R).as_quat(), S)
    q = q_average(np.array(list(Q)))
    r = Rotation.from_quat(q).as_matrix()

    P = map(lambda s : s.P, S)
    p = np.average(np.array(list(P)), axis=0)

    return System(p, r, bs)


def print_system(system):
    print(f"Base station {system.bs} @ {system.P}")


def probe_position(Sbs_ref, Sbs0_other, Sbs1_other):
    # Find transform from other ref frame to global using bs0
    Sother_g = transform_from_ref_x_to_r_same_bs(Sbs0_other, Sbs_ref)

    # The meassurement of base station 1 in the other ref frame, converted to global
    Sbs1mOther_g = transform_x_to_r(Sbs1_other, Sother_g)

    return Sbs1mOther_g


def combine_measurements(measurements):

    # Convert measurements to System format
    measurements = [
        [System(geom.origin, geom.rotation_matrix, bs_id) for bs_id, geom in m.items()]
        for m in measurements
    ]

    print()
    ref = measurements[0][0]

    result = [ref]
    found = [ref.bs]

    not_done = True

    while not_done:
        not_done = False
        samples = {}

        print("--- iteration")

        for measurement in measurements:
            # Find a reference system in this measurement
            from_bs = None
            from_sys = None
            from_sys_g = None
            for system in measurement:
                bs = system.bs
                for sys in result:
                    if sys.bs == bs:
                        from_bs = bs
                        from_sys = system
                        from_sys_g = sys
                        break
                if from_bs is not None:
                    break

            # Transform all base stations in this measurement to
            # the global system, unless we already have a result for a
            # particular base station
            if from_bs is not None:
                print(f"Using {from_bs} as reference")
                for sys in measurement:
                    if sys is not from_sys:
                        if sys.bs not in found:
                            s = probe_position(from_sys_g, from_sys, sys)
                            from_to = (from_bs, sys.bs)
                            if not from_to in samples:
                                samples[from_to] = []
                            samples[from_to].append(s)
            else:
                not_done = True

        # Average over all samples sets
        for from_to, sample_set in samples.items():
            # Averaging might create suboptimal solution
            new_sys = system_average(sample_set)
            result.append(new_sys)
            found.append(new_sys.bs)

    geometries = {}
    for sys in result:
        print_system(sys)
        geo = LighthouseBsGeometry()
        geo.rotation_matrix = sys.R
        geo.origin = sys.P
        geo.valid = True
        geometries[sys.bs] = geo

    return geometries