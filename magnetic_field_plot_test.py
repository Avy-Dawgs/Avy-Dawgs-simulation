import numpy as np
from magnetic_field import *
import matplotlib.pyplot as plt
import matplotlib as mpl
from beacon import *

# Parameters
HALFGRID_XY = 50    # half of horizontal axes
LLIMIT_Z = -25      # lower limit z axis
HLIMIT_Z = 25       # high limit z axis

SPACE_PER_POINT = 5

POINTS_PER_DIM = int(HALFGRID_XY * 2 / SPACE_PER_POINT)


def main():
    '''
    Entry point.
    '''

    # list of beacons
    beacon_list: list[Beacon] = []

    # ADD beacons
    beacon_list.append(Beacon(0, 0, 0, (1, 0, 0)))
    beacon_list.append(Beacon(16, 16, 0, (1, 0, 0)))
    beacon_list.append(Beacon(-16, -16, 0, (1, 0, 0)))

    # make 3D axes
    ax = plt.figure().add_subplot(projection='3d')

    x = y = np.linspace(-HALFGRID_XY, HALFGRID_XY, POINTS_PER_DIM)
    z = np.linspace(LLIMIT_Z, HLIMIT_Z, POINTS_PER_DIM)
    
    # make meshgrid 
    x_mesh, y_mesh, z_mesh = np.meshgrid(x, y, z)

    # generate vector components
    u, v, w = B_beacons_cartesion(beacon_list, x_mesh, y_mesh, z_mesh)

    # plot vectors
    ax.quiver(x_mesh, y_mesh, z_mesh, u, v, w, length=HALFGRID_XY/POINTS_PER_DIM, normalize=True)
    mpl.rcParams["axes3d.mouserotationstyle"] = "azel"          # matlab style mouse controls

    # plot each beacon as a dot
    for b in beacon_list:
        ax.plot(b.x, b.y, b.z, "r-o")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
    
    # SURFACE PLOT

    mag = B_magnitude_calc(u, v, w)
    # X, Y = np.meshgrid(np.linspace(-HALFGRID_XY, HALFGRID_XY, 10),
                          # np.linspace(-HALFGRID_XY, HALFGRID_XY, 10))
    x_mesh, y_mesh = np.meshgrid(x, y)

    mag_xy = mag[:, :, 5]
    rssi_dB = 20 * np.log10(mag_xy)

    ax = plt.figure().add_subplot(projection='3d')
    ax.plot_surface(x_mesh, y_mesh, rssi_dB, vmin=rssi_dB.min() * 2)

    # show plot
    plt.show()


if __name__ == "__main__": 
    main()
