import gym
import gym_panda
import math
import numpy as np
import pybullet as p
env = gym.make('panda-v0')

done = False
error = 0.01
rotate_error=0.015
fingers = 1
info1 = [0.7, 0, 0.1]

k_p = 10
k_d = 1
dt = 1./240. # the default timestep in pybullet is 240 Hz
t = 0

for i_episode in range(20):
    observation = env.reset()
    fingers = 1
    for t in range(100):
        env.render()
        dx = info1[0]-observation[0]
        dy = info1[1]-observation[1]
        target_z = info1[2]
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
        info1 = info['object_position']
        #modification, because 'info' is a dictionary
        if done:
            print("Grasp episode finished after {} timesteps".format(t+1))
            break
    if info1[2]>0.45:
        #'YZX' [x:math.pi, y:-math.pi*3/4, z:-math.pi / 2.]
        # target = [-math.pi/4., 0., -math.pi / 2.]
        # 'YZX' [x:math.pi, y:-math.pi/6., z:-math.pi / 2.]
        target_orientations = []
        for i in range(10):
            target_orientations.append( \
                p.multiplyTransforms(env.ee_position, \
                                     p.getQuaternionFromEuler( \
                                         [math.pi, 0, -3*math.pi / 2.]), \
                                     [0, 0, 0], p.getQuaternionFromEuler \
                                         ([0, 0, -(i + 1) * math.pi / 10]))[1])

        target_orientation = target_orientations[0]
        target_position = env.ee_position
        t1 = 0
        t = 0
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
                if (t==1 and t1==0) or (t==0 and t1!=0):
                    # each small step for 2 degree
                    dp_x = (target_orientation[0] - info2[0])/9.
                    dp_y = (target_orientation[1] - info2[1])/9.
                    dp_z = (target_orientation[2] - info2[2])/9.
                    dp_w = (target_orientation[2] - info2[3])/9.
                if t>=16:
                    dp_x = target_orientation[0] - info2[0]
                    dp_y = target_orientation[1] - info2[1]
                    dp_z = target_orientation[2] - info2[2]
                    dp_w = target_orientation[2] - info2[3]
                ddx = target_position[0] - info1[0]
                ddy = target_position[1] - info1[1]
                ddz = target_position[2] - info1[2]
                pd_x = 0.2*k_p * ddx + k_d * ddx / dt
                pd_y = 0.2*k_p * ddy + k_d * ddy / dt
                pd_z = 0.2*k_p * ddz + k_d * ddz / dt
            action=[pd_x,pd_y,pd_z,fingers,dp_x,dp_y,dp_z,dp_w]
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
            elif euler_error[0]<=rotate_error and euler_error[1]<=rotate_error and euler_error[2]<=rotate_error and \
                euler_error[3]<=rotate_error and pos_error[0]<=error and pos_error[1]<=error and pos_error[2]<=error:
                print("Rotate ",str(info2)," episode finished after ",t+1," timesteps")
                t = 0
                t1 += 1
                if t1 <10:
                    target_orientation = target_orientations[t1]
                elif t1 == 10:
                    target_orientation = p.getQuaternionFromEuler([-math.pi / 2, 0, -math.pi / 2.])
                    target_position = [env.ee_position[0] + 0.1, env.ee_position[1], env.ee_position[2] + 0.15]

                elif t1 == 11:
                    target_orientation = p.getQuaternionFromEuler([-math.pi / 3, 0, -math.pi / 2.])
                    target_position = [env.ee_position[0] + 0.1, env.ee_position[1], env.ee_position[2] + 0.3]
                else:
                    break

            if t>=300:
                break
        print(info2)
env.close()
