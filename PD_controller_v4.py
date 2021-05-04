import gym
import gym_panda
import math
import numpy as np
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
        action = [pd_x,pd_y,pd_z,fingers,0,0,0]
        observation, reward, done, info = env.step(action)
        info1 = info['object_position']
        #modification, because 'info' is a dictionary
        if done:
            print("Grasp episode finished after {} timesteps".format(t+1))
            break
    print(info1)
    if info1[2]>0.45:
        # target = [-1., 0., -math.pi / 2.]
        target = [-math.pi/2., 0., -math.pi / 2.]
        for t in range(300):
            env.render()
            if t==0:
                dyaw=0
                dpitch=0
                droll=0
            else:
                dyaw = target[0]-info2[0]
                dpitch = target[1]-info2[1]
                droll = target[2]-info2[2]
            action=[0,0,0,fingers,dyaw,dpitch,droll]
            observation, reward, done, info = env.step(action)
            info1 = info['object_position']
            info2 = info['Orientation']
            dx = info1[0] - observation[0]
            dy = info1[1] - observation[1]
            dz = info1[2] - observation[2]
            euler_error=sum(abs(np.array(target) - np.array(info2)))
            if abs(dx) >= 5*error or abs(dy) >= 5*error or abs(dz) >= 5*error:
                break
            elif euler_error<=error and (abs(dx) < 5*error and abs(dy) < 5*error and abs(dz) < 5*error):
                print("Rotate ",str(info2)," episode finished after ",t+1," timesteps")
                break
        print(info2)
env.close()
