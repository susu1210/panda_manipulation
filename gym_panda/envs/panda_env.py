import gym
from gym import error, spaces, utils
from gym.utils import seeding

import os
import pybullet as p
import pybullet_data
import math
import numpy as np
import random
from pybullet_object_models import ycb_objects
from Load_Object_URDF import LoadObjectURDF
MAX_EPISODE_LEN = 20*100

class PandaEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.step_counter = 0
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90, cameraPitch=-14, cameraTargetPosition=[0.60, 0.0, 0.45])
        self.action_space = spaces.Box(np.array([-1]*4), np.array([1]*4))
        self.observation_space = spaces.Box(np.array([-1]*5), np.array([1]*5))
        self.maxFingerForce = 20.0
        self.object="000"
        self.storage_folder=os.path.join(os.path.abspath(os.path.dirname(os.getcwd())), "3d_object_reconstruction", "Data")
        self.record_end = False
    def step(self, action):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)

        jointPoses = p.calculateInverseKinematics(self.pandaUid, 11, action[1], action[2])[0:7]
        
        p.setJointMotorControlArray(self.pandaUid, self.joints, p.POSITION_CONTROL,
                                    list(jointPoses))
        p.setJointMotorControlArray(self.pandaUid, [9, 10], p.POSITION_CONTROL, action[3])

        p.stepSimulation()
        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])

        self.step_counter += 1

        if self.step_counter > MAX_EPISODE_LEN:
            reward = 0
            done = True
        else:
            reward = 0
            done = False

        self.observation = state_robot  + state_fingers
        return np.array(self.observation).astype(np.float32), reward, done

    def activate(self):
        while self.step_counter < MAX_EPISODE_LEN:
            p.setJointMotorControlArray(self.pandaUid, [9, 10], p.POSITION_CONTROL, [0, 0])
            p.stepSimulation()
            contacts = p.getContactPoints(self.pandaUid, self.objectUid)
            if len(contacts)>=3:
                break
            self.step_counter += 1
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])
        return state_fingers

    def reset(self):
        self.step_counter = 0
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
        urdfRootPath=pybullet_data.getDataPath()
        p.setGravity(0,0,-10)
        planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,-0.65])

        rest_poses = [0,-0.215,0,-2.57,0,2.356,2.356,0.08,0.08]
        self.pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True)
        num_joints = p.getNumJoints(self.pandaUid)
        joints = [p.getJointInfo(self.pandaUid, i) for i in range(num_joints)]
        self.joints = [j[0] for j in joints if j[2] == p.JOINT_REVOLUTE]
        for i in range(7):
            p.resetJointState(self.pandaUid,i, rest_poses[i])
        p.resetJointState(self.pandaUid, 9, 0.08)
        p.resetJointState(self.pandaUid,10, 0.08)
        currentPosition = p.getLinkState(self.pandaUid, 11)[0]
        jointPoses = p.calculateInverseKinematics(self.pandaUid, 11,
                                                  [currentPosition[0],currentPosition[1]+0.02,currentPosition[2]],
                                                  p.getLinkState(self.pandaUid, 11)[1])[0:7]

        p.setJointMotorControlArray(self.pandaUid, list(range(7)) + [9, 10], p.POSITION_CONTROL,
                                    list(jointPoses) + 2 * [0.08])

        tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.65])
        trayUid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.5,0,0])
        self.objectUid = LoadObjectURDF(self.object)
        p.stepSimulation()


        state_robot = p.getLinkState(self.pandaUid, 11)[0]

        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])

        self.observation = state_robot + state_fingers
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        return np.array(self.observation).astype(np.float32)


    def render(self, mode='human'):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.5, 0, 0.15],
                                                          distance=.7,
                                                          yaw=0,
                                                          pitch=-20,
                                                          roll=0,
                                                          upAxisIndex=2)

        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(960) /720,
                                                     nearVal=0.1,
                                                     farVal=100.0)
        (_, _, px, pd, _) = p.getCameraImage(width=960,
                                            height=720,
                                            viewMatrix=view_matrix,
                                            projectionMatrix=proj_matrix,
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def storage(self):
        # result=p.getDebugVisualizerCamera(self)
        width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget =p.getDebugVisualizerCamera(self)

        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.60, 0.0, 0.45],
                                                          distance=0.5,
                                                          yaw=90,
                                                          pitch=-14,
                                                          roll=0,
                                                          upAxisIndex=2)
        far = 100.0
        near = 0.1
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(640) /480,
                                                     nearVal=near,
                                                     farVal=far)
        (_, _, px, pd, pseg) = p.getCameraImage(width=640,
                                            height=480,
                                            viewMatrix=view_matrix,
                                            projectionMatrix=proj_matrix,
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)


        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (480,640, 4))
        rgb_array = rgb_array[:, :, :3]
        cad = rgb_array.copy()
        cad[pd >= 1.0] = np.array([0, 0, 0], dtype=np.uint8)
        pd[pd >= 1.0] = 0

        m= pseg.max()
        pmask = pseg.copy()
        pseg[pseg == m] = 255
        pseg[pseg != 255] = 0
        pmask[pseg == 255] = 1
        pmask[pseg != 255] = 0
        pseg = np.array(pseg, dtype=np.uint8)
        pmask = np.array(pmask, dtype=np.uint8)


        m , mm = pd.max(), pd.min()
        pd = pd * 65535
        pd = pd.astype(np.uint16)

        return rgb_array,cad,pd,pseg,pmask
    def _get_state(self):
        return self.observation

    def close(self):
        p.disconnect()
