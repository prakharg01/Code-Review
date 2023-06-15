import numpy as np
import math
from hardware_env.stoch3_kinematics import Serial2RKinematics
import matplotlib.pyplot as plt
import csv
import gym
from gym import spaces
import socket
import struct


class HardwareEnv(gym.Env):

    def __init__(self) -> None:
        super(HardwareEnv).__init__()

        self.serverAddressPort = ("10.42.0.50", 20001)
        self.bufferSize = 32  # 4 * 8 bytes will change this
        self.UDPClientSocket = socket.socket(
            family=socket.AF_INET, type=socket.SOCK_DGRAM)

        self.link_length_1 = 0.297
        self.link_length_2 = 0.28

        self.link_mass_1 = 0.20
        self.link_mass_2 = 0.20

        self.leg_step_length = 0.2
        self.z_foot = -0.4
        self.step_height = 0.1

        self._obs_dim = 7  # [phase, pos, vel. cmd_torques]
        # np.array([np.pi/2] * self._obs_dim)
        observation_high = np.array([np.inf] * self._obs_dim)
        observation_low = -observation_high
        self.observation_space = spaces.Box(observation_low, observation_high)

        self._action_dim = 2
        action_high = np.array([10] * self._action_dim)
        self.action_space = spaces.Box(-action_high, action_high)

        self.phase = 0
        self.reset_ = 0
        self.last_action = np.array([0, 0])

    def reset(self) -> None:

        self.phase = 0
        self.last_action = np.array([0, 0])
        reset_pos = [0, -0.25]

        '''
        Reset to this position or angles
        1. calculate commands for resetting
        2. apply commands
        3. return base observation
        '''

        action = np.array([0, 0])
        self.reset_ = 1

        success = False

        # while ~success:
        #    success = self.apply_action(action)
        success = self.apply_action(action)

        self.update_sensor_data()

        next_obs = self.get_observation()

        return next_obs

    def step(self, action):

        success = False

        # while ~success:
        #    success = self.apply_action(action)
        success = self.apply_action(action)

        self.update_sensor_data()

        next_obs = self.get_observation()
        reward = self.calc_reward()
        terminal = self.is_state_terminal()

        return next_obs, reward, terminal, {}

    def get_observation(self):
        '''
        collect and process information from sensor data
        '''

        next_pos = self.current_pos  # calculate from motor encoders
        next_vel = self.current_vel  # calculate from motor encoders

        next_obs = np.concatenate(
            ([self.phase], next_pos, next_vel, self.last_action)).ravel()

        return next_obs

    def calc_reward(self):
        '''
        calculate reward based on tracking error
        reward = np.exp(np.linalg.norm(current_ee_pos - desired_ee_pos))
        '''
        serial_2r_ = Serial2RKinematics()
        current_pos = serial_2r_.forwardKinematics(self.current_pos)

        current_ee_pos = current_pos
        desired_ee_pos = np.array([0, 0])  # should be a function of the phase

        reward = np.exp(np.linalg.norm(current_ee_pos - desired_ee_pos))

        return reward

    def is_state_terminal(self):
        '''
        calc is state is terminal
        1. if jerk very large?
        2. if tracking error is more than threshold
        return true or false
        '''

        return False

    def apply_action(self, action):
        '''
        write your hardware communication code here
        return if command transfer is successfull 
        '''
        send_array = [action[0], action[1], 0]
        send_array_bytes = struct.pack('ddd', *send_array)
        self.UDPClientSocket.sendto(send_array_bytes, self.serverAddressPort)

        return True

    def update_sensor_data(self):
        '''
        write your hardware communication code here
        collect data from all the sensors
        '''
        msgFromServer = self.UDPClientSocket.recvfrom(self.bufferSize)
        new_sensor_data = struct.unpack('dddd', msgFromServer[0])
        pos_0 = new_sensor_data[0]
        pos_1 = new_sensor_data[1]
        vel_0 = new_sensor_data[2]
        vel_1 = new_sensor_data[3]

        self.current_pos = np.array([pos_0, pos_1])
        self.current_vel = np.array([vel_0, vel_1])
