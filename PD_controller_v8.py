import gym
import gym_panda
import math
import numpy as np
import pybullet as p
import os
import cv2
import pandas as pd
from Load_Target_PosOrient import *
env = gym.make('panda-v0')
env.object="YcbChipsCan"
done = False
error = 0.01
rotate_error=0.02
fingers = 0
info1 = [0.431545, 0, 0.234615]#[0.7, 0, 0.1]
k_p = 2#10
k_d = 1#1
dt = 1./240. # the default timestep in pybullet is 240 Hz
t = 0

for i_episode in range(1):
    count=0
    # info1 = [0.431545, -0.0, 0.234615]
    info1 = [0.7, 0, 0.1]
    observation = env.reset()
    fingers =1
    #  grasping procedure
    for t in range(100):
        env.render()
        dx = info1[0]-observation[0]
        dy = info1[1]-observation[1]
        if not count:
            target_z = info1[2]
            dz = target_z - observation[2]
        if abs(dx) < error and abs(dy) < error and abs(dz) < error:
            fingers = 0
        if env.object=="YcbChipsCan":
            if abs(dx) < error and abs(dy) < error and abs(info1[2] - observation[2]) < error :
                target_z = 0.5
                count += 1
        elif env.object=="YcbGelatinBox":
            if abs(dx) < error and abs(dy) < error and abs(dz) < error:
                fingers = 0
            # if (observation[3] + observation[4]) < error + 2*fingers and fingers == 0.009:
            if (observation[3] + observation[4]) < error + 0.015 and fingers == 0:
                target_z = 0.55
        elif env.object=="YcbBanana":
            if abs(dx) < error and abs(dy) < error and abs(dz) < error:
                fingers = 0.0125
            if abs(observation[3] - fingers) <= error and abs(observation[4] - fingers) <= error and fingers == 0.0125:
                    target_z = 0.5
        else:
            if abs(dx) < error and abs(dy) < error and abs(dz) < error:
                fingers = 0
            if (observation[3]+observation[4])<error+0.02 and fingers==0:
                target_z = 0.5


        dz = target_z-observation[2]
        pd_x = k_p*dx + k_d*dx/dt
        pd_y = k_p*dy + k_d*dy/dt
        pd_z = k_p*dz + k_d*dz/dt
        action = [pd_x,pd_y,pd_z,fingers,0,0,0,0]
        observation, reward, done, info = env.step(action)
        info1 = info['ObjectPosition']
        if done:
            print("Grasp episode finished after {} timesteps".format(t+1))
            break
    # rotation procedure
    filenames = []
    LinkPositions = []
    LinkOrientations = []
    ObjectPositions = []
    ObjectOrientations = []
    LinkR = []
    ObjectR = []
    if info1[2]>0.45:
        if not os.path.exists(env.storage_folder+"/"+env.object+"/"):
            os.makedirs(env.storage_folder + "/" + env.object + "/")
            os.makedirs(env.storage_folder + "/" + env.object + "/" + "color/")
            os.makedirs(env.storage_folder + "/" + env.object + "/" + "cad/")
            os.makedirs(env.storage_folder + "/" + env.object + "/" + "depth/")
            os.makedirs(env.storage_folder + "/" + env.object + "/" + "annotations/")
            os.makedirs(env.storage_folder + "/" + env.object + "/" + "mask/")
        folder = env.storage_folder + "/" + env.object + "/"
        target_positions,target_orientations=LoadTargetPosOrient(env.ee_position)
        target_orientation=target_orientations[0]
        target_position = target_positions[0]
        t1 = 0
        t = 0
        iter = 0
        while(True):

            env.render()

            if t==0 and t1==0:
                dp_x = 0
                dp_y = 0
                dp_z = 0
                dp_w = 0
                ddx = 0
                ddy = 0
                ddz = 0
            else:
                if (t1==0 and t==1) or (t1!=0 and t==0) :
                    step_max=[(target_orientation[0] - info2[0])/8.,(target_orientation[1] - info2[1])/8.,
                          (target_orientation[2] - info2[2])/8.,(target_orientation[3] - info2[3])/8.]
                if abs(target_orientation[0] - info2[0] )< abs(step_max[0]) or abs(target_orientation[1] - info2[1] )< abs(step_max[1])\
                    or abs(target_orientation[2] - info2[2] )< abs(step_max[2]) or abs(target_orientation[3] - info2[3]) < abs(step_max[3]):
                    dp_x = target_orientation[0] - info2[0]
                    dp_y = target_orientation[1] - info2[1]
                    dp_z = target_orientation[2] - info2[2]
                    dp_w = target_orientation[3] - info2[3]
                else:
                    dp_x = step_max[0]
                    dp_y = step_max[1]
                    dp_z = step_max[2]
                    dp_w = step_max[3]
                ddx = target_position[0] - info1[0]
                ddy = target_position[1] - info1[1]
                ddz = target_position[2] - info1[2]
                pd_x = 0.2*k_p * ddx + 0.2*k_d * ddx / dt
                pd_y = 0.2*k_p * ddy + 0.2*k_d * ddy / dt
                pd_z = 0.2*k_p * ddz + 0.2*k_d * ddz / dt
            if env.object=="YcbGelatinBox":
                fingers=0.01
            elif  env.object=="YcbBanana":
                fingers=0.0125
            action=[pd_x,pd_y,pd_z,fingers,dp_x,dp_y,dp_z,dp_w]

            observation, reward, done, info = env.step(action)
            info1 = info['ObjectPosition']
            info2 = info['LinkOrientation']
            info3 = info['ObjectOrientation']
            # storage
            filenames.append(iter)
            LinkPositions.append(observation[0:3]) # recording end effector postion
            LinkOrientations.append(info2)  # recording end effector orientation
            LinkR.append(p.getMatrixFromQuaternion(info2))
            ObjectPositions.append(info1)
            ObjectOrientations.append(info3)
            ObjectR.append(p.getMatrixFromQuaternion(info3))
            rgb_filename = folder + 'color/%s.jpg' % str(iter)
            cad_filename = folder + 'cad/%s.jpg' % str(iter)
            depth_filename = folder + 'depth/%s.png' % str(iter)
            annotation_filename= folder + 'annotations/%s.png' % str(iter)
            mask_filename = folder + 'mask/%s.png' % str(iter)
            rgb, cad, depth, annotation, mask = env.storage()
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cad = cv2.cvtColor(cad, cv2.COLOR_RGB2BGR)
            cv2.imwrite(rgb_filename, rgb)
            cv2.imwrite(cad_filename, cad)
            cv2.imwrite(depth_filename, depth)
            cv2.imwrite(annotation_filename, annotation)
            cv2.imwrite(mask_filename, mask)

            dx = info1[0] - observation[0]
            dy = info1[1] - observation[1]
            dz = info1[2] - observation[2]
            euler_error=abs(np.array(target_orientation) - np.array(info2))
            pos_error=abs(np.array(target_position) - np.array(info1))
            t += 1
            # whether the gripper is still grasping the object
            if abs(dx) >= 5*error or abs(dy) >= 5*error or abs(dz) >= 5*error:
                break
            elif euler_error[0]<=rotate_error and euler_error[1]<=rotate_error and euler_error[2]<=rotate_error and \
                euler_error[3]<=rotate_error and pos_error[0]<=3*error and pos_error[1]<=3*error and pos_error[2]<=3*error:
                print("Rotate ",str(info2)," episode finished after ",t+1," timesteps")
                t = 0
                t1 += 1
                if t1 <len(target_orientations):
                    target_orientation = target_orientations[t1]
                    target_position = target_positions[t1]
                else:
                    print("succesful rotations")
                    break

            if t>=300:
                break
            iter += 1
        print(info2)

        robot_joints = pd.DataFrame({'filenames': filenames,
                                   'LinkPositions': LinkPositions,
                                   'LinkOrientations': LinkOrientations,
                                     'LinkRotationMatrices':LinkR,
                                     'ObjectPositions': ObjectPositions,
                                     'ObjectOrientations': ObjectOrientations,
                                     'ObjectRotationMatrices': ObjectR
                                     })
        robot_joints.to_csv(folder + '/robot_joints.csv', index=False)
        width, height, _, _, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(env)
        projMat=p.computeProjectionMatrixFOV(fov=60,
                                             aspect=float(640) / 480,
                                             nearVal=0.1,
                                             farVal=100.0)
        viewMat= p.computeViewMatrixFromYawPitchRoll(
                                         cameraTargetPosition=[0.60, 0.0, 0.45],
                                         distance=0.5,
                                         yaw=90,
                                         pitch=-14,
                                         roll=0,
                                         upAxisIndex=2)
        camera_intrinsic = pd.DataFrame({'width': 640,
                                     'height': 480,
                                     'cameraUp': str(cameraUp),
                                     'camForward': str(camForward),
                                     'horizon': str(horizon),
                                     'vertical': str(vertical),
                                     'dist': dist,
                                     'camTarget':str(camTarget),
                                     'fov' : 60,
                                     'farVal' : 100.0,
                                     'nearVal' : 0.1,
                                     'viewMat': str(viewMat),
                                     'projMat': str(projMat),
                                     }, index=[0])
        camera_intrinsic.to_csv(folder + '/camera_intrinsic.csv', index=False)
env.close()
