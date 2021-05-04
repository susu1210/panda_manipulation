import pybullet as p
import math
import numpy as np
from convert_matrix_to_quaternion import *
from convert_euler_to_matrix import *
from convert_euler_to_quatenion import *
target = [math.pi / 8, math.pi, math.pi / 2.]
R=convertEuler2Matrix([math.pi/2.,math.pi,math.pi/8.],'ZYX')
print(R)
print(convertMatrix2Quaternion(R))
print(convertEuler2Quaternion([math.pi/2.,math.pi,math.pi/8.],'ZYX'))
print(p.getQuaternionFromEuler(target))
print(p.getMatrixFromQuaternion(p.getQuaternionFromEuler(target)))

R=convertEuler2Matrix([-math.pi/2.,0,math.pi],'ZYX')
print(R)
print(convertMatrix2Quaternion(R))
print(p.getQuaternionFromEuler([math.pi , 0, -math.pi / 2.]))

print(p.getMatrixFromQuaternion(p.getQuaternionFromEuler([math.pi , 0, -math.pi / 2.])))


R=convertEuler2Matrix([math.pi/2.,-math.pi,0.],'ZYX')
# R=convertEuler2Matrix([-math.pi / 2.,0, math.pi],'ZYX')
print(R)
print(convertMatrix2Quaternion(R))
print(convertEuler2Quaternion([math.pi/2.,-math.pi,0],'ZYX'))
print(p.getQuaternionFromEuler([0, -math.pi, math.pi / 2.]))
print(p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0, -math.pi, math.pi / 2.])))

print(convertEuler2Quaternion([math.pi/2.,math.pi,0],'XZY'))
print(convertEuler2Quaternion([math.pi,0,-math.pi/2.],'ZYX'))
print(p.getEulerFromQuaternion([0.8858620768096295, -0.46388374623464557, -0.0011613394795372148, -0.007674772990426973]))

target_orientation= p.multiplyTransforms([0.69,0.0,0.45], p.getQuaternionFromEuler( [math.pi, 0, -math.pi / 2.]), [0, 0, 0], p.getQuaternionFromEuler ([0, 0, - math.pi ]))[1]

last= p.getEulerFromQuaternion(target_orientation)
target_orientation = convertEuler2Quaternion([-math.pi/4.,math.pi*3/2.,math.pi],'YZX')
print(target_orientation)
print((p.getEulerFromQuaternion([0.3535533845424652, 0.3535533845424652, 0.6123723983764648, 0.6123724579811096])))
print(p.getEulerFromQuaternion([0.3718129028246925, 0.37146711109043046, 0.6435617153736802, 0.5564132180102226]))




