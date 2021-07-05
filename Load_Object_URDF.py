import pybullet as p
import pybullet_data
import os
import math
from pybullet_object_models import ycb_objects
def LoadObjectURDF(ObjectName):
    print(ycb_objects.getDataPath())
    urdfRootPath = pybullet_data.getDataPath()
    state_object = [0.6, 0.0, 0.05]
    stateOrientation=p.getQuaternionFromEuler([0,0,0])
    if not ObjectName.isalpha():
        objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/"+ObjectName+"/"+ObjectName+".urdf"),
                               basePosition=state_object,baseOrientation=stateOrientation)
    elif ObjectName=='YcbChipsCan':
        state_object = [0.431545, 0.113, 0.264615]
        # state_object = [0.7, 0.13, 0.05]
        # stateOrientation=p.getQuaternionFromEuler([math.pi / 2., 0, 0])
        stateOrientation=p.getQuaternionFromEuler([0.,-math.pi,math.pi/2.])

        # objectUid = p.loadURDF(os.path.join(ycb_objects_path, ObjectName, "poisson / textured.obj"),
        #                        basePosition=state_object, baseOrientation=stateOrientation)
        objectUid = p.loadURDF(os.path.join(ycb_objects.getDataPath(), ObjectName, "model1.urdf"),
                                    basePosition=state_object, baseOrientation=stateOrientation)
    elif ObjectName == 'YcbBanana':
        stateOrientation = p.getQuaternionFromEuler([0, 0, math.pi / 2.])
        objectUid = p.loadURDF(os.path.join(ycb_objects.getDataPath(), ObjectName, "model.urdf"),
                               basePosition=state_object,
                               baseOrientation= stateOrientation)
        p.changeDynamics(objectUid, -1, lateralFriction=0.8, spinningFriction=0.3, rollingFriction=0.3)
    elif ObjectName == 'YcbCrackerBox':
        state_object = [0.5, 0.015, 0.11]
        stateOrientation = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
        objectUid = p.loadURDF(os.path.join(ycb_objects.getDataPath(), ObjectName, "model.urdf"),
                               basePosition=state_object, baseOrientation=stateOrientation)
    elif ObjectName == 'YcbGelatinBox':
        state_object = [0.5, -0.035, 0.05]
        stateOrientation=p.multiplyTransforms([0,0,0],p.getQuaternionFromEuler(
            [-math.pi/2.,0,0]),state_object,p.getQuaternionFromEuler([0,0,math.pi/4.]))[1]
        objectUid = p.loadURDF(os.path.join(ycb_objects.getDataPath(), ObjectName, "model.urdf"),
                               basePosition=state_object, baseOrientation=stateOrientation)
        p.changeDynamics(objectUid, -1, lateralFriction=1.0, rollingFriction=0)
    elif ObjectName == 'YcbTomatoSoupCan':
        state_object = [0.5, -0., 0.11]
        objectUid = p.loadURDF(os.path.join(ycb_objects.getDataPath(), ObjectName, "model.urdf"),
                               basePosition=state_object,)
        p.changeDynamics(objectUid, -1, lateralFriction=1.0, spinningFriction=0.8, rollingFriction=1.0)
    elif ObjectName == 'YcbMustardBottle':
        state_object = [0.5, 0.015, 0.11]
        stateOrientation = p.getQuaternionFromEuler([0, 0, 0])
        objectUid = p.loadURDF(os.path.join(ycb_objects.getDataPath(), ObjectName, "model.urdf"),
                               basePosition=state_object, baseOrientation=stateOrientation)
    elif ObjectName == 'YcbPottedMeatCan':
        state_object = [0.5, 0.015, 0.11]
        stateOrientation = p.getQuaternionFromEuler([0, 0, -math.pi/3])
        objectUid = p.loadURDF(os.path.join(ycb_objects.getDataPath(), ObjectName, "model.urdf"),
                               basePosition=state_object, baseOrientation=stateOrientation)
        p.changeDynamics(objectUid, -1, lateralFriction=0.8, spinningFriction=0, rollingFriction=0)
    elif ObjectName == "YcbSugarBox":
        state_object = [0.5, 0.015, 0.11]
        stateOrientation = p.getQuaternionFromEuler([math.pi/2., 0, 0])
        objectUid = p.loadURDF(os.path.join(ycb_objects.getDataPath(), ObjectName, "model.urdf"),
                               basePosition=state_object, baseOrientation=stateOrientation)
        p.changeDynamics(objectUid, -1, lateralFriction=1.0, spinningFriction=0., rollingFriction=0.)
    else:
        objectUid = p.loadURDF(os.path.join(ycb_objects.getDataPath(), ObjectName, "model.urdf"),
                                basePosition=state_object)
    return objectUid