from typing import Tuple
import numpy as np

class Beacon:
    '''
    Represents location and orientation of a beacon.
    '''
    def __init__(self, x: float, y: float, z: float, m: Tuple[float, float, float]) -> None:
        '''
        Initiate a beacon given cartesion coordinates, dipole moment vector. 

        Dipole moment vector is normalized on instanciation.
        '''
        self.x = x 
        self.y = y 
        self.z = z

        self.m = np.array(m) / np.linalg.norm(np.array(m))
