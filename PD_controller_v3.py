import gym
import gym_panda
import math
import numpy as np
import pybullet as p

env = gym.make('panda-v0')

done = False
error = 0.01
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
        target = p.getQuaternionFromEuler([-math.pi/4, 0., -math.pi / 2.])
        t1 = 0
        t = 0
        while (True):
            env.render()
            if t==0:
                dq_x=0
                dq_y=0
                dq_z=0
                dq_w = 0
            else:
                dq_x = target[0]-info2[0]
                dq_y = target[1]-info2[1]
                dq_z = target[2]-info2[2]
                dq_w = target[3] - info2[3]
            action=[0,0,0,fingers,dq_x,dq_y,dq_z,dq_w]
            observation, reward, done, info = env.step(action)
            info1 = info['object_position']
            info2 = info['Orientation']
            dx = info1[0] - observation[0]
            dy = info1[1] - observation[1]
            dz = info1[2] - observation[2]
            euler_error=sum(abs(np.array(target) - np.array(info2)))
            t += 1
            if abs(dx) >= 5*error or abs(dy) >= 5*error or abs(dz) >= 5*error:
                break
            elif euler_error<=0.1 and (abs(dx) < 5*error and abs(dy) < 5*error and abs(dz) < 5*error):
                print("Rotate ",str(target)," episode finished after ",t+1," timesteps")
                t = 0
                if t1 == 0:
                    target = p.getQuaternionFromEuler([-math.pi / 2, 0, -math.pi / 2.])
                elif t1 == 1:
                    target = p.getQuaternionFromEuler([-math.pi / 2, 0, -math.pi / 4.])
                elif t1 == 2:
                    target = p.getQuaternionFromEuler([-math.pi / 2, 0, -math.pi*3 / 4.])
                else:
                    break
                t1 +=1
            if t >= 300:
                break
        print(info2)
        print(p.getEulerFromQuaternion(info2))
env.close()
