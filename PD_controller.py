import gym
import gym_panda
import math
import numpy as np
env = gym.make('panda-v0')

done = False
error = 0.01
fingers = 1
info = [0.7, 0, 0.1]

k_p = 10
k_d = 1
dt = 1./240. # the default timestep in pybullet is 240 Hz
t = 0

for i_episode in range(20):
    observation = env.reset()
    fingers = 1
    for t in range(100):
        env.render()
        dx = info[0]-observation[0]
        dy = info[1]-observation[1]
        target_z = info[2]
        if abs(dx) < error and abs(dy) < error and abs(dz) < error:
            fingers = 0
        if (observation[3]+observation[4])<error+0.02 and fingers==0:
            target_z = 0.5
        dz = target_z-observation[2]
        pd_x = k_p*dx + k_d*dx/dt
        pd_y = k_p*dy + k_d*dy/dt
        pd_z = k_p*dz + k_d*dz/dt
        action = [pd_x,pd_y,pd_z,fingers]
        observation, reward, done, info = env.step(action)
        #modification, because 'info' is a dictionary
        info = info['object_position']
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
    if info[2]>0.45:
        for t in range(100):
            env.render()
            if t==0:
                target=[-math.pi/2,0,-math.pi/2.]
                dyaw=target[0]
                dpitch=target[1]
                droll=target[2]
            else:
                dyaw = target[0]-info[0]
                dpitch = target[1]-info[1]
                droll = target[2]-info[2]
            action=[dyaw,dpitch,droll]
            info=env.rotate(action)
            info=info['Orientation']
            if sum(abs(np.array(target)-np.array(info)))<=0.7:
                break

env.close()
