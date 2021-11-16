import pybullet as p
import pybullet_data
import os
import math
from pybullet_object_models import ycb_objects
BasePosAndOriDict = {
    "general" : [
        [0.6, 0.0, 0.03],
        [0,0,0]
    ],
    "YcbChipsCan": [
        [0.431545, 0.113, 0.264615],
        [0.,-math.pi,math.pi/2.]
    ],
    "YcbBanana": [
        [0.6, 0.0, 0.05],
        [0, 0, math.pi / 2.]
    ],
    "YcbCrackerBox": [
        [0.5, 0.015, 0.11],
        [math.pi / 2, 0, 0]
    ],
    "YcbGelatinBox": [
        [0.5, -0.035, 0.05],
        [-1.5707964369299257, 0.7853982841747686, -4.56194285191881e-08]
    ],
    "YcbTomatoSoupCan": [
        [0.5, -0., 0.11],
        [0, 0, 0]
    ],
    "YcbMustardBottle": [
        [0.5, 0.015, 0.11],
        [0, 0, 0]
    ],
    "YcbPottedMeatCan": [
        [0.5, 0.015, 0.11],
        [0, 0, -math.pi/3]
    ],
    "YcbSugarBox": [
        [0.5, 0.015, 0.11],
        [math.pi/2., 0, 0]
    ],
}
def LoadObjectURDF(ObjectName):
    print(ycb_objects.getDataPath())
    urdfRootPath = pybullet_data.getDataPath()
    state_object = BasePosAndOriDict["general"][0]
    stateOrientation=p.getQuaternionFromEuler(BasePosAndOriDict["general"][1])
    if not ObjectName.isalpha():
        objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/"+ObjectName+"/"+ObjectName+".urdf"),
                               basePosition=state_object,baseOrientation=stateOrientation)
    else:
        if ObjectName in BasePosAndOriDict:
            state_object = BasePosAndOriDict[ObjectName][0]
            stateOrientation=p.getQuaternionFromEuler(BasePosAndOriDict[ObjectName][1])
        objectUid = p.loadURDF(os.path.join(ycb_objects.getDataPath(), ObjectName, "model.urdf"),
                                    basePosition=state_object, baseOrientation=stateOrientation)
    if ObjectName == 'YcbBanana' or ObjectName == 'YcbTennisBall':
        p.changeDynamics(objectUid, -1, lateralFriction=0.8, spinningFriction=0., rollingFriction=0.)
    if ObjectName == 'YcbTomatoSoupCan':
        p.changeDynamics(objectUid, -1, lateralFriction=1.0, spinningFriction=0.8, rollingFriction=1.0)
    if ObjectName == 'YcbPottedMeatCan':
        p.changeDynamics(objectUid, -1, lateralFriction=0.8, spinningFriction=0, rollingFriction=0)
    if ObjectName == "YcbSugarBox" or ObjectName == 'YcbGelatinBox' or ObjectName == 'YcbChipsCan':
        p.changeDynamics(objectUid, -1, lateralFriction=1.0, spinningFriction=0., rollingFriction=0.)

    return objectUid