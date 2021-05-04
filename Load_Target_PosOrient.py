import pybullet as p
import math
from math import sqrt
def LoadTargetPosOrient(ee_position):
    target_orientations = []
    target_positions = []
    # target_orientations.append(
    #     p.multiplyTransforms([0, 0, 0],
    #                          p.getQuaternionFromEuler([0,  math.pi / 4 , 0]),
    #                          ee_position,
    #                          p.getQuaternionFromEuler([math.pi, 0, -math.pi / 2.]))[1])
    # target_positions.append([ee_position[0] +0.1 , ee_position[1], ee_position[2]])
    # target_orientations.append(p.getQuaternionFromEuler([math.pi, 0, -math.pi / 2.]))
    # target_positions.append(ee_position)
    for i in range(6):
        target_orientations.append(
            p.multiplyTransforms([0, 0, 0],
                                 p.getQuaternionFromEuler([0, 0, (i + 1) * math.pi / 4]),
                                 ee_position,
                                 p.getQuaternionFromEuler([math.pi, 0, -math.pi / 2.]))[1])
        target_positions.append(ee_position)
    for i in range(2):
        target_orientations.append(
            p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler
            ([0, -(i + 1) * math.pi / 4, 0.]),
                                 ee_position, target_orientations[3])[1])
    target_positions.append(
        [ee_position[0] + 0.1, ee_position[1], ee_position[2] + 0.1 * (sqrt(2) - 1)])
    target_positions.append(
        [ee_position[0] + 0.1 * sqrt(2), ee_position[1], ee_position[2] + 0.15 * sqrt(2)])
    # target_orientations.append(
    #     p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler
    #     ([0, -math.pi * 2. / 3., 0.]), target_positions[-1],
    #                          target_orientations[3])[1])
    # target_positions.append(
    #     [ee_position[0] + 0.1 * sqrt(2), ee_position[1], ee_position[2] + 0.15 * sqrt(2) + 0.1])
    return target_positions,target_orientations