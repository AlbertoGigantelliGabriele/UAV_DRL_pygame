import os
from math import sin, cos, pi, sqrt
from random import randrange
import numpy as np
import gymnasium as gym
from gym import spaces
import pygame
from pygame.locals import *
import math 

def calculate_distance_to_target(x, y, xt, yt):
    return sqrt((xt - x) ** 2 + (yt - y) ** 2) / 500

def calculate_velocity(xd, yd):
    return sqrt(xd**2 + yd**2)

def calculate_angle_to_target(x, y, xt, yt):
    return np.arctan2(yt - y, xt - x)

def calculate_angle_target_and_velocity(x, y, xd, yd, xt, yt):
    return np.arctan2(yt - y, xt - x) - np.arctan2(yd, xd)

def calculate_angle_to_up(a):
    return a / 180 * pi

def calculate_angle_velocity(ad):
    return ad

class DEnv(gym.Env):

    def __init__(self, render_every_frame, mouse_target):
        super(DEnv, self).__init__()

        self.render_every_frame = render_every_frame
        self.mouse_target = mouse_target

        # Initialize Pygame, load sprites
        pygame.init()
        self.screen = pygame.display.set_mode((800, 800)) # TODO: leggere informazioni schermo
        self.FramePerSec = pygame.time.Clock()
        
        self.player = pygame.image.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../images/drones/drone2/drone2.png"))
        self.player.convert()
        self.player = pygame.transform.scale(self.player, (80, 80))

        self.target = pygame.image.load(os.path.join(os.path.dirname(os.path.abspath(__file__)),  "../images/target/simple-targets/target3.png"))
        self.target.convert()
        self.target = pygame.transform.scale(self.target, (50, 50))

        pygame.font.init()

        # Physics constants
        self.FPS = 60
        self.thruster_amplitude = 0.04
        self.angular_velocity = 0  # Nuova variabile, controllare

        # Initialize variables
        (self.a, self.ad, self.add) = (0, 0, 0)
        (self.x, self.xd, self.xdd) = (400, 0, 0)
        (self.y, self.yd, self.ydd) = (400, 0, 0)
        self.xt = randrange(200, 600)
        self.yt = randrange(200, 600)

        # Initialize game variables
        self.target_counter = 0
        self.reward = 0
        self.time = 0
        self.time_limit = 20
        if self.mouse_target is True:
            self.time_limit = 1000

        # 2 action thrust x and thrust y in float values between -1 and 1
        # self.action_space = gym.spaces.Box(low=-1, high=1, shape=(2,))
        
        # 3 action: thrust x, thrust y, and rotation in float values between -1 and 1
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32)

        # 8 observations: angle_to_up, velocity, angle_velocity, distance_to_target, angle_to_target, angle_target_and_velocity, distance_to_target
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(6,))

    def reset(self, seed=42, options=None):
        super().reset(seed=seed) # Gymnasium != Gym
        
        # Reset variables
        (self.a, self.ad, self.add) = (0, 0, 0)
        (self.x, self.xd, self.xdd) = (400, 0, 0)
        (self.y, self.yd, self.ydd) = (400, 0, 0)
        self.xt = randrange(200, 600)
        self.yt = randrange(200, 600)

        self.target_counter = 0
        self.reward = 0
        self.time = 0
        self.angular_velocity = 0  # Nuova variabile

        return self.get_obs(), {}

    def get_obs(self) -> np.ndarray:
        """
        Calculates the observations

        Returns:
            np.ndarray: The normalized observations:
            - angle_to_up : angle between the drone and the up vector
            - velocity : velocity of the drone
            - angle_velocity : angle of the velocity vector
            - distance_to_target : distance to the target
            - angle_to_target : angle between the drone and the target
            - angle_target_and_velocity : angle between the to_target vector and the velocity vector    
        """
        
        angle_to_up = calculate_angle_to_up(self.a)
        velocity = calculate_velocity(self.xd, self.yd)
        angle_velocity = self.ad
        distance_to_target = calculate_distance_to_target(self.x, self.y, self.xt, self.yt)
        angle_to_target = calculate_angle_to_target(self.x, self.y, self.xt, self.yt)
        # Angle between the to_target vector and the velocity vector
        angle_target_and_velocity = calculate_angle_target_and_velocity(self.x, self.y, self.xd, self.yd, self.xt, self.yt)
        
        return np.array([
                angle_to_up,
                velocity,
                self.angular_velocity,  # Nuova variabile
                distance_to_target,
                angle_to_target,
                angle_target_and_velocity,
            ]).astype(np.float32)

    def step(self, action):

        self.reward = 0.0
        (action0, action1, rotation_action) = (action[0], action[1], action[2])

        # Act every 5 frames
        for _ in range(5):
            self.time += 1 / 60

            if self.mouse_target is True:
                self.xt, self.yt = pygame.mouse.get_pos()

            # Initialize accelerations
            self.xdd = 0
            self.ydd = 0
            self.add = 0

            # Calcola la spinta come nel secondo codice
            self.xdd += math.cos(math.radians(self.a)) * action0 * self.thruster_amplitude - math.sin(math.radians(self.a)) * action1 * self.thruster_amplitude
            self.ydd += math.sin(math.radians(self.a)) * action0 * self.thruster_amplitude + math.cos(math.radians(self.a)) * action1 * self.thruster_amplitude

            # Aggiorna la velocità angolare e la rotazione
            self.angular_velocity = rotation_action  # Nuova variabile
            self.a += self.angular_velocity  # Aggiorna la rotazione

            self.xd += self.xdd
            self.yd += self.ydd
            self.ad += self.add
            self.x += self.xd
            self.y += self.yd
            # self.a += self.ad

            dist = sqrt((self.x - self.xt) ** 2 + (self.y - self.yt) ** 2) #/500

            # Reward per step survived
            self.reward += 1 / 60
            # Penalty according to the distance to target
            self.reward -= dist / (100 * 60)

            if dist < 50:
                # Reward if close to target
                self.xt = randrange(200, 600)
                self.yt = randrange(200, 600)
                self.reward += 100

            # If out of time
            if self.time > self.time_limit:
                done = True
                break

            # If too far from target (crash)
            elif dist > 1000:
                self.reward -= 1000
                done = True
                break

            else:
                done = False

            if self.render_every_frame is True:
                self.render("yes")

        info = {}
        truncated = False
        
        return (
            self.get_obs(),
            self.reward,
            done,
            truncated,
            info,
        )

    def render(self, mode):
        pygame.event.get()
        self.screen.fill(0)
        self.screen.blit(
            self.target,
            (
                self.xt - int(self.target.get_width() / 2),
                self.yt - int(self.target.get_height() / 2),
            ),
        )
        player_copy = pygame.transform.rotate(self.player, self.a)
        self.screen.blit(
            player_copy,
            (
                self.x - int(player_copy.get_width() / 2),
                self.y - int(player_copy.get_height() / 2),
            ),
        )

        #print('target raggiunti:', str(self.target_counter))
        #print('tempo:', str(int(self.time)))

        pygame.display.update()
        self.FramePerSec.tick(self.FPS)

    def close(self):
        pass

import training