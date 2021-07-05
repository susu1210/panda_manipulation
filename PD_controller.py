import time
import os
import gym
import gym_panda
import reflexxes
import pybullet as p
import transformations as tf
import math
import numpy as np
import cv2
import pandas as pd
class MovementData:
    def __init__(self, id):
        self.mov_id=id
        self.currentPosition = [0.0, 0.0, 0.0]*2
        self.currentVelocity = [0.0, 0.0, 0.0]*2
        self.currentAcceleration = [0.0, 0.0, 0.0]*2
        self.targetPosition = [0.0, 0.0, 0.0]*2
        self.targetVelocity = [0.0, 0.0, 0.0]*2
        self.min_sync_time = 0.0 # in rml, min_sync_time <= time_reach_target_pos_and_vel; if min_sync_time is larger than required, then motion will be less greedy and take time of min_sync; Otherwise, if required time is larger than given min_sync, required time is taken to ensure reach target

class Agent:
    def __init__(self, env, hz=240):
        self.gen = reflexxes.extra.PositionTrajectoryGenerator(
            number_of_dofs=3,
            cycle_time=1/float(hz),
            max_velocity=[5.0, 5.0, 5.0],
            max_acceleration=[10.0, 10.0, 10.0],
            max_jerk=[20.0, 20.0, 20.0])
        self.grasp_com_offset = [0.0, 0.0, -0.015]
        self.iter = 0
    def get_obj_position(self, obj_id):
        return p.getBasePositionAndOrientation(obj_id)[0]

    def get_obj_orientation(self, obj_id):
        return p.getBasePositionAndOrientation(obj_id)[1]

    def get_tip_position(self, env):
        return p.getLinkState(env.pandaUid, 11)[0]

    def get_tip_orientation(self, env):
        return p.getLinkState(env.pandaUid, 11)[1]

    def gen_motion_list(self, motion_data):
        self.gen.current_position = motion_data.currentPosition[:]
        self.gen.current_velocity = motion_data.currentVelocity[:]
        self.gen.current_acceleration = motion_data.currentAcceleration[:]

        pos_list = [motion_data.currentPosition[:]]
        vel_list = [motion_data.currentVelocity[:]]
        acc_list = [motion_data.currentAcceleration[:]]

        # generate trajectory
        # gen.trajectory(target_pos, target_vel, min_sync_time)
        for pos, vel, acc in self.gen.trajectory(motion_data.targetPosition, motion_data.targetVelocity, motion_data.min_sync_time):
            pos_list.append(pos)
            vel_list.append(vel)
            acc_list.append(acc)

        return pos_list, vel_list, acc_list
    def recording(self, env):
        if self.iter == 0:
            if not os.path.exists(env.storage_folder+"/"+env.object+"/"):
                os.makedirs(env.storage_folder + "/" + env.object + "/")
                os.makedirs(env.storage_folder + "/" + env.object + "/" + "color/")
                os.makedirs(env.storage_folder + "/" + env.object + "/" + "cad/")
                os.makedirs(env.storage_folder + "/" + env.object + "/" + "depth/")
                os.makedirs(env.storage_folder + "/" + env.object + "/" + "annotations/")
                os.makedirs(env.storage_folder + "/" + env.object + "/" + "mask/")
            self.folder = env.storage_folder + "/" + env.object + "/"
            self.filenames = []
            self.LinkPositions = []
            self.LinkOrientations = []
            self.LinkR = []
            self.ObjectPositions = []
            self.ObjectOrientations = []
            self.ObjectR = []

        self.filenames.append(self.iter)
        self.LinkPositions.append(self.get_tip_position(env))  # recording end effector postion
        self.LinkOrientations.append(self.get_tip_orientation(env))  # recording end effector orientation
        self.LinkR.append(p.getMatrixFromQuaternion(self.LinkOrientations[-1]))
        self.ObjectPositions.append(self.get_obj_position(env.objectUid))
        self.ObjectOrientations.append(self.get_obj_orientation(env.objectUid))
        self.ObjectR.append(p.getMatrixFromQuaternion(self.ObjectOrientations[-1]))
        rgb_filename = self.folder + 'color/%s.jpg' % str(self.iter)
        cad_filename = self.folder + 'cad/%s.jpg' % str(self.iter)
        depth_filename = self.folder + 'depth/%s.png' % str(self.iter)
        annotation_filename = self.folder + 'annotations/%s.png' % str(self.iter)
        mask_filename = self.folder + 'mask/%s.png' % str(self.iter)
        rgb, cad, depth, annotation, mask = env.storage()
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cad = cv2.cvtColor(cad, cv2.COLOR_RGB2BGR)
        cv2.imwrite(rgb_filename, rgb)
        cv2.imwrite(cad_filename, cad)
        cv2.imwrite(depth_filename, depth)
        cv2.imwrite(annotation_filename, annotation)
        cv2.imwrite(mask_filename, mask)
        self.iter += 1

        if env.record_end:
            robot_joints = pd.DataFrame({'filenames': self.filenames,
                                         'LinkPositions': self.LinkPositions,
                                         'LinkOrientations': self.LinkOrientations,
                                         'LinkRotationMatrices': self.LinkR,
                                         'ObjectPositions': self.ObjectPositions,
                                         'ObjectOrientations': self.ObjectOrientations,
                                         'ObjectRotationMatrices': self.ObjectR
                                         })
            robot_joints.to_csv(self.folder + '/robot_joints.csv', index=False)

if __name__ == "__main__":
    RECORD = False
    env = gym.make('panda-v0')
    # object to be grasped
    env.object = "YcbTomatoSoupCan"
    # prior: grasping position offset w.r.t center of mass of object
    grasp_offset_dict = {
        "YcbPottedMeatCan": [0, 0.005, 0.015],
        "YcbGelatinBox": [0, +0.003, 0.022],
        "YcbMustardBottle": [0, 0, 0.08],
        "YcbTomatoSoupCan": [0, 0.007, 0.025],
        "YcbCrackerBox": [0, -0.01, 0.035],
        "YcbSugarBox": [0, 0, 0.0],
    }
    grasp_offset = grasp_offset_dict[env.object]
    agent = Agent(env)

    
    observation = env.reset()

    while not env.is_static():
        p.stepSimulation()
        time.sleep(0.001)
    fingers = 1
    obj_position = agent.get_obj_position(env.objectUid)
    prepick_position = [x+y for x,y in zip(obj_position, [0, 0, 0.15])]
    grasp_position = [x + y for x, y in zip(obj_position, grasp_offset)]
    init_tip_pose = agent.get_tip_position(env)
    prepick_data = MovementData('prepick')
    prepick_data.currentPosition = init_tip_pose
    prepick_data.targetPosition = prepick_position
    prepick_data.min_sync_time = 2.

    pick_data = MovementData('pick')
    pick_data.currentPosition = prepick_position
    pick_data.targetPosition = grasp_position
    pick_data.min_sync_time = 1

    prepick_pos, prepick_vel, prepick_acc = agent.gen_motion_list(prepick_data)
    pick_pos, pick_vel, pick_acc = agent.gen_motion_list(pick_data)

    pick_group_pos = prepick_pos[:-1] + pick_pos  # remove duplicated pos
    pick_group_vel = prepick_vel[:-1] + pick_vel  # remove duplicated vel
    pick_group_acc = prepick_acc[:-1] + pick_acc  # remove possible duplicated acc to match size
    pick_group_traj_time = np.linspace(0, agent.gen.cycle_time * len(pick_group_pos),
                                       len(pick_group_pos)).tolist()
    pick_time = len(prepick_pos) * agent.gen.cycle_time

    # Compute gripper orientation and rotation increments
    init_tip_ori = agent.get_tip_orientation(env)  # quartenion


    fingers = [0.1, 0.1]
    for i in range(len(pick_group_pos)):
        action = ['pick', pick_group_pos[i], init_tip_ori, fingers]
        observation, reward, done = env.step(action)


    fingers = env.activate()

    lift_position = [x + y for x, y in zip(grasp_position, [0, 0, 0.4])]
    lift_data = MovementData('lift')
    lift_data.currentPosition = grasp_position
    lift_data.targetPosition = lift_position
    lift_data.min_sync_time = 3
    lift_pos, lift_vel, lift_acc = agent.gen_motion_list(lift_data)

    for i in range(len(lift_pos)):
        action = ['lift', lift_pos[i], init_tip_ori, fingers]
        observation, reward, done = env.step(action)

    # start recording
    ee_position = agent.get_tip_position(env)
    for j in range(6):
        rotate_pos = [ee_position]* 120 # hz=240, lasting 0.5 second.
        rotate_group_ori = []
        pre_ori = agent.get_tip_orientation(env)
        for i in range(len(rotate_pos)):
            rotate_group_ori.append(p.getQuaternionFromEuler(
                [0., -np.pi, np.pi / 2.+ np.pi/4*i/len(rotate_pos) + np.pi/4*j]
            ))  # quaternion
        for i in range(len(rotate_pos)):
            action = ['rotate', rotate_pos[i], rotate_group_ori[i], fingers]
            observation, reward, done = env.step(action)
            if RECORD: agent.recording(env)
    # [0., -np.pi, np.pi / 2.]
    [0, -np.pi, 2 * np.pi]
    for j in range(2):
        rotate_pos = [ee_position]* 120 # hz=240, lasting 0.5 second.
        rotate_group_ori = []
        pre_ori = agent.get_tip_orientation(env)
        for i in range(len(rotate_pos)):
            rotate_group_ori.append(p.getQuaternionFromEuler(
                [0., -np.pi, 2 * np.pi - np.pi / 4 * i / len(rotate_pos) - np.pi / 4 * j]
            ))  # quaternion
        for i in range(len(rotate_pos)):
            action = ['rotate', rotate_pos[i], rotate_group_ori[i], fingers]
            observation, reward, done = env.step(action)
            if RECORD: agent.recording(env)

    offset = [
        [0.1, 0, 0.1*(np.sqrt(2) - 1) ],
        [0.1 * np.sqrt(2), 0, 0.15 * np.sqrt(2)]
    ]
    rotate_position = ee_position
    for j in range(2):
        pre_position = rotate_position
        rotate_position = [x + y for x, y in zip(ee_position, offset[j])]
        rotate_data = MovementData('rotate')
        rotate_data.currentPosition = pre_position
        rotate_data.targetPosition = rotate_position
        rotate_data.min_sync_time = 0.5
        rotate_pos, rotate_vel, rotate_acc = agent.gen_motion_list(rotate_data)

        rotate_group_ori = []
        pre_ori = agent.get_tip_orientation(env)
        for i in range(len(rotate_pos)):
            rotate_group_ori.append(p.multiplyTransforms(positionA=[0, 0, 0], orientationA=p.getQuaternionFromEuler(
                [0, -np.pi / 4 * i / len(rotate_pos), 0.]
            ), positionB=[0, 0, 0], orientationB=pre_ori)[1]
                                    )  # quaternion
        for i in range(len(rotate_pos)):
            action = ['rotate', rotate_pos[i], rotate_group_ori[i], fingers]
            observation, reward, done = env.step(action)
            if RECORD: agent.recording(env)

    env.record_end = True
    if RECORD: agent.recording(env)

    env.close()
