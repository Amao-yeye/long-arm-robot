import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d


def plot(Array):
    plt.figure()
    ax = plt.axes(projection='3d')
    # Data for a three-dimensional line
    x1 = Array[0]
    y1 = Array[1]
    z1 = Array[2]
    x2 = Array[3]
    y2 = Array[4]
    z2 = Array[5]

    ax.plot3D(x1, y1, z1, 'b')  # 绘制空间曲线
    ax.plot3D(x2, y2, z2, 'r--')
    ax.plot3D(x1[1::2], y1[1::2], z1[1::2], 'b.')
    # ax.plot3D(x2[1::2], y2[1::2], z2[1::2], 'r.')

    ax.set_xlabel('x')
    ax.set_xlim(-7, 7)
    ax.set_ylabel('y')
    ax.set_ylim(-7, 7)
    ax.set_zlabel('z')
    ax.set_zlim(0, 7)
    plt.show()
