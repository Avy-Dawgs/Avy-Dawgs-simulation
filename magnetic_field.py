from typing import Tuple
import numpy as np
from beacon import *


@np.vectorize
def B_vector_calc_cartesion(x: float, y: float, z: float, mx: float = 1, my: float = 0, mz: float = 0) -> Tuple[float, float, float]:
    '''
    Calculate magnetic field in cartesion coordinates.

    x, y, z: coordinates 
    mx, my, mz: dipole moment vector components (must be normalized)
    '''

    # distance 
    r = np.sqrt(x**2 + y**2 + z**2)

    # deal with edge case where exactly on origin
    if r == 0:
        return 0, 0, 0

    # scaling factor
    mu0 = 4 *  np.pi * 1e-7
    scale = mu0/(4*np.pi)

    # position vector
    pos = np.array([x, y, z])

    # normalize positon vector
    rhat = pos / np.linalg.norm(pos)

    # dipole moment vector
    m = np.array([mx, my, mz])

    retval = scale * (3 * np.dot(m, rhat) * rhat - m) / r**3

    return retval[0], retval[1], retval[2]


def B_beacons_cartesion(beacon_list: list[Beacon], x: float, y: float, z: float) -> Tuple[float, float, float]:
    '''
    Calculate magnetic field vector at given cartesion coordinate given list of beacons.
    '''
    
    x_sum, y_sum, z_sum = 0, 0, 0
    
    for b in beacon_list:
        # get intermediate results
        xi, yi, zi = B_vector_calc_cartesion(x - b.x, y - b.y, z - b.z, b.m[0], b.m[1], b.m[2])

        x_sum += xi 
        y_sum += yi 
        z_sum += zi

    return x_sum, y_sum, z_sum
