import gym
import gym_panda
import math
import numpy as np
import pybullet as p
from Load_Target_PosOrient import *
env = gym.make('panda-v0')
env.object="YcbChipsCan"
done = False
error = 0.01
rotate_error=0.015
fingers = 1
info1 = [0.431545, -0.03, 0.234615]#[0.7, 0, 0.1]
k_p = 2#10
k_d = 1#1
dt = 1./240. # the default timestep in pybullet is 240 Hz
t = 0

for i_episode in range(20):
    count=0
    observation = env.reset()
    fingers = 1
    #  grasping procedure
    for t in range(100):
        env.render()
        dx = info1[0]-observation[0]
        dy = info1[1]-observation[1]
        if not count:
            target_z = info1[2]
        if abs(dx) < error and abs(dy) < error and abs(dz) < error:
            fingers = 0
        if env.object=="YcbChipsCan":
            if abs(dx) < 0.005 and abs(dy) < 0.005 and abs(info1[2] - observation[2]) < 0.005:
                target_z = 0.5
                count += 1
        elif env.object=="YcbGelatinBox":
            if (observation[3] + observation[4]) < error + 0.01 and fingers == 0:
                target_z = 0.5
        else:
            if (observation[3]+observation[4])<error+0.02 and fingers==0:
                target_z = 0.5


        dz = target_z-observation[2]
        pd_x = k_p*dx + k_d*dx/dt
        pd_y = k_p*dy + k_d*dy/dt
        pd_z = k_p*dz + k_d*dz/dt
        action = [pd_x,pd_y,pd_z,fingers,0,0,0,0]
        observation, reward, done, info = env.step(action)
        info1 = info['object_position']
        if done:
            print("Grasp episode finished after {} timesteps".format(t+1))
            break
    # rotation procedure
    if info1[2]>0.45:
        fingers=0
        target_positions,target_orientations=LoadTargetPosOrient(env.ee_position)
        # target_orientation=target_orientations[0]
        target_orientation = p.getEulerFromQuaternion(target_orientations[0])
        target_position = target_positions[0]
        t1 = 0
        t = 0
        while(True):
            env.render()
            if t==0 and t1==0:
                # dp_x = 0
                # dp_y = 0
                # dp_z = 0
                # dp_w = 0
                droll = 0
                dpitch = 0
                dyaw = 0
                ddx = 0
                ddy = 0
                ddz = 0
            else:
                # if (t1==0 and t==1) or (t1!=0 and t==0) :
                #     step_max=[(target_orientation[0] - info2[0])/4.,(target_orientation[1] - info2[1])/4.,
                #           (target_orientation[2] - info2[2])/4.,(target_orientation[3] - info2[3])/4.]
                # if abs(target_orientation[0] - info2[0] )< abs(step_max[0]) or abs(target_orientation[1] - info2[1] )< abs(step_max[1])\
                #     or abs(target_orientation[2] - info2[2] )< abs(step_max[2]) or abs(target_orientation[3] - info2[3]) < abs(step_max[3]):
                #     dp_x = target_orientation[0] - info2[0]
                #     dp_y = target_orientation[1] - info2[1]
                #     dp_z = target_orientation[2] - info2[2]
                #     dp_w = target_orientation[3] - info2[3]
                # else:
                #     dp_x = step_max[0]
                #     dp_y = step_max[1]
                #     dp_z = step_max[2]
                #     dp_w = step_max[3]
                if (t1==0 and t==1) or (t1!=0 and t==0) :
                    step_max=[(target_orientation[0] - info2[0])/4.,(target_orientation[1] - info2[1])/4.,
                          (target_orientation[2] - info2[2])/4.]
                if abs(target_orientation[0] - info2[0] )< abs(step_max[0]) or abs(target_orientation[1] - info2[1] )< abs(step_max[1])\
                    or abs(target_orientation[2] - info2[2] )< abs(step_max[2]) :
                    droll = target_orientation[0] - info2[0]
                    dpitch = target_orientation[1] - info2[1]
                    dyaw = target_orientation[2] - info2[2]
                else:
                    droll = step_max[0]
                    dpitch = step_max[1]
                    dyaw = step_max[2]
                ddx = target_position[0] - info1[0]
                ddy = target_position[1] - info1[1]
                ddz = target_position[2] - info1[2]
                pd_x = 0.2*k_p * ddx + 0.2*k_d * ddx / dt
                pd_y = 0.2*k_p * ddy + 0.2*k_d * ddy / dt
                pd_z = 0.2*k_p * ddz + 0.2*k_d * ddz / dt
            # action=[pd_x,pd_y,pd_z,fingers,dp_x,dp_y,dp_z,dp_w]
            action = [pd_x, pd_y, pd_z, fingers, droll, dpitch, dyaw]
            Link7=p.getLinkState(env.pandaUid,6)[1]
            Link7_Euler=p.getEulerFromQuaternion(Link7)
            Joint7=p.getJointState(env.pandaUid,6)
            observation, reward, done, info = env.step(action)
            info1 = info['object_position']
            info2 = info['Orientation']
            dx = info1[0] - observation[0]
            dy = info1[1] - observation[1]
            dz = info1[2] - observation[2]
            euler_error=abs(np.array(target_orientation) - np.array(info2))
            pos_error=abs(np.array(target_position) - np.array(info1))
            t += 1
            # whether the gripper is still grasping the object
            if abs(dx) >= 5*error or abs(dy) >= 5*error or abs(dz) >= 5*error:
                break
            # elif euler_error[0]<=rotate_error and euler_error[1]<=rotate_error and euler_error[2]<=rotate_error and \
            #     euler_error[3]<=rotate_error and pos_error[0]<=3*error and pos_error[1]<=3*error and pos_error[2]<=3*error:
            #     print("Rotate ",str(info2)," episode finished after ",t+1," timesteps")
            #     t = 0
            #     t1 += 1
            #     if t1 <6:
            #         target_orientation = target_orientations[t1]
            #         target_position = target_positions[t1]
            elif euler_error[0] <= rotate_error and euler_error[1] <= rotate_error and euler_error[2] <= rotate_error and \
                pos_error[0] <= 3 * error and pos_error[1] <= 3 * error and pos_error[2] <= 3 * error:
                print("Rotate ", str(info2), " episode finished after ", t + 1, " timesteps")
                t = 0
                t1 += 1
                if t1 < 6:
                    target_orientation = p.getEulerFromQuaternion(target_orientations[t1])
                    target_position = target_positions[t1]
                else:
                    print("succesful rotations")
                    break

            if t>=300:
                break
        print(info2)
env.close()
