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
        self.storage_folder="~/3d_object_reconstruction/Data"
    def step(self, action):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)

        if action[4]==0 and action[5]==0 and action[6]==0 and action[7]==0:
            newOrientation = p.getQuaternionFromEuler([0.,-math.pi,math.pi/2.])#[roll, pitch, yaw]
            dv = 0.005
            dx = action[0] * dv
            dy = action[1] * dv
            dz = action[2] * dv
            fingers = action[3]

            currentPose = p.getLinkState(self.pandaUid, 11)
            currentPosition = currentPose[0]
            newPosition = [currentPosition[0] + dx,
                           currentPosition[1] + dy,
                           currentPosition[2] + dz]
        else:
            if action[0]==0 and action[1]==0 and action[2]==0:
                newPosition = self.ee_position
            else:
                dv = 0.005
                dx = action[0] * dv
                dy = action[1] * dv
                dz = action[2] * dv
                currentPosition = p.getLinkState(self.pandaUid, 11)[0]
                newPosition = [currentPosition[0] + dx,
                               currentPosition[1] + dy,
                               currentPosition[2] + dz]
            currentOrientation = p.getLinkState(self.pandaUid, 11)[1]
            dv = 0.8
            dq_x = action[4] * dv
            dq_y = action[5] * dv
            dq_z = action[6] * dv
            dq_w = action[6] * dv
            newOrientation = [currentOrientation[0] + dq_x,
                              currentOrientation[1] + dq_y,
                              currentOrientation[2] + dq_z,
                              currentOrientation[3] + dq_w]

            fingers = action[3]
            # set Fingers with MaxForce
            p.setJointMotorControlArray(self.pandaUid, [9, 10], p.TORQUE_CONTROL, [self.maxFingerForce, self.maxFingerForce])
            # p.setJointMotorControlArray(self.pandaUid, [9, 10], p.TORQUE_CONTROL, [10, 10])


        jointPoses = p.calculateInverseKinematics(self.pandaUid,11,newPosition, newOrientation)[0:7]

        p.setJointMotorControlArray(self.pandaUid, list(range(7))+[9,10], p.POSITION_CONTROL, list(jointPoses)+2*[fingers])

        p.stepSimulation()
        if self.object=='YcbChipsCan':
            state_object, _ = p.getBasePositionAndOrientation(self.objectUid)
            state_object = np.array(state_object).astype(np.float32)
            state_object[1] = state_object[1] - 0.02
        else:
            state_object, _ = p.getBasePositionAndOrientation(self.objectUid)

        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        # state_orient = p.getEulerFromQuaternion(p.getLinstate_orientState(self.pandaUid, 11)[1])
        state_orient = p.getLinkState(self.pandaUid, 11)[1]
        # pos_error=state_robot-state_object
        # if abs(pos_error[0])<0.005 and abs(pos_error[1])<0.005 and abs(pos_error[2])<0.005:
        #     state_fingers=(0,0)
        # else:
        #     state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])
        state_fingers = (p.getJointState(self.pandaUid, 9)[0], p.getJointState(self.pandaUid, 10)[0])

        if state_object[2]>0.45:
            reward = 1
            done = True
            # record the current end-effector position
            if action[4]==0 and action[5]==0 and action[6]==0:
                self.ee_position = p.getLinkState(self.pandaUid, 11)[0]
        else:
            reward = 0
            done = False

        self.step_counter += 1

        if self.step_counter > MAX_EPISODE_LEN:
            reward = 0
            #self.ee_position = p.getLinkState(self.pandaUid, 11)[0]
            done = True

        #info = {'object_position': state_object}
        info = {'ObjectPosition': state_object, 'LinkOrientation': state_orient , 'ObjectOrientation': p.getBasePositionAndOrientation(self.objectUid)[1]}

        self.observation = state_robot + state_fingers
        return np.array(self.observation).astype(np.float32), reward, done, info


    def reset(self):
        self.step_counter = 0
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
        urdfRootPath=pybullet_data.getDataPath()
        p.setGravity(0,0,-10)
        planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,-0.65])

        rest_poses = [0,-0.215,0,-2.57,0,2.356,2.356,0.08,0.08]
        self.pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True)
        for i in range(7):
            p.resetJointState(self.pandaUid,i, rest_poses[i])
        p.resetJointState(self.pandaUid, 9, 0.08)
        p.resetJointState(self.pandaUid,10, 0.08)
        # print(p.getNumJoints(self.pandaUid))  # numofjoints=12
        # for i in range(p.getNumJoints(self.pandaUid)):
        #    print(i,p.getJointInfo(self.pandaUid,i))
        # result:
        # joint 0-6: JOINT_REVOLUTE(rotational); joint 7,8,11: JOINT_FIXED;
        # joint 9,10: finger_joint1 and finger_joint_2 JOINT_PRISMATIC(linear)
        #for i in range(p.getNumJoints(self.pandaUid)):
        #    print(i,p.getJointState(self.pandaUid, i)[0], p.getJointState(self.pandaUid, i)[1])
        #    print(i,p.getLinkState(self.pandaUid, i)[0],p.getLinkState(self.pandaUid, i)[1])

        tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.65])
        trayUid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.5,0,0])
        self.objectUid = LoadObjectURDF(self.object)

        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        gripper_orient=p.getEulerFromQuaternion(p.getLinkState(self.pandaUid, 11)[1])
        # getJointState
        # inputs:body unique id, link index in range [0..getNumJoints(bodyUniqueId)]
        # 1st output: The position value of this joint.
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
        # view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7, 0, 0.65],
        #                                                   distance=.7,
        #                                                   yaw=0,
        #                                                   pitch=0,
        #                                                   roll=0,
        #                                                   upAxisIndex=2)

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
        # view_matrix , proj_matrix = result[2] , result[3]
        # p.resetDebugVisualizerCamera(cameraDistance=0.7, cameraYaw=90, cameraPitch=-10,
        #                              cameraTargetPosition=[0.60, 0.0, 0.45])
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
        # filename = self.storage_folder + "/" + self.object + "/" + "matrices.txt"
        # d = {'projection': proj_matrix, 'view': view_matrix}
        # s = str(d)
        # f = open(filename, 'w')
        # f.writelines(s)
        # f.close()
        # pd = far * near / (far - (far - near) * pd)

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

        # import cv2
        # cv2.imshow('rgb',rgb_array)
        # cv2.imshow('depth', pd)
        # pseg = np.array(pseg, dtype=np.uint8)
        # cv2.imshow('segmenation', pseg)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        return rgb_array,cad,pd,pseg,pmask
    def _get_state(self):
        return self.observation

    def close(self):
        p.disconnect()
