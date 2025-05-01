import numpy as np
from magnetic_field import *
import matplotlib.pyplot as plt
import matplotlib as mpl
from beacon import *

# Parameters
HALFGRID_XY = 25    # half of horizontal axes
LLIMIT_Z = -25      # lower limit z axis
HLIMIT_Z = 25       # high limit z axis


def main():
    '''
    Entry point.
    '''

    # list of beacons
    beacon_list: list[Beacon] = []

    # ADD beacons
    # BEACON_LIST.append(Beacon(0, 0, 0, [1, 0, 0]))
    # BEACON_LIST.append(Beacon(16, 16, 8, [1, 1, 0]))
    # BEACON_LIST.append(Beacon(-12, -12, -8, [0, 0, -1]))
    beacon_list.append(Beacon(0, 0, 0, (1.0, 1.0, 0.0)))

    # make 3D axes
    ax = plt.figure().add_subplot(projection='3d')
    
    # make meshgrid 
    x, y, z = np.meshgrid(np.linspace(-HALFGRID_XY, HALFGRID_XY, 10),
                          np.linspace(-HALFGRID_XY, HALFGRID_XY, 10),
                          np.linspace(LLIMIT_Z, HLIMIT_Z, 10))

    # generate vector components
    u, v, w = B_beacons_cartesion(beacon_list, x, y, z)

    # plot vectors
    ax.quiver(x, y, z, u, v, w, length=HALFGRID_XY/10, normalize=True)
    mpl.rcParams["axes3d.mouserotationstyle"] = "azel"          # matlab style mouse controls

    # plot each beacon as a dot
    for b in beacon_list:
        ax.plot(b.x, b.y, b.z, "r-o")
        ax.set_xlabel("x")
        ax.set_ylabel("y")

    # show plot
    plt.show()


if __name__ == "__main__": 
    main()
