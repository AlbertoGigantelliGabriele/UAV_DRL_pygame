import os
from pygame.locals import *
from stable_baselines3 import SAC

class Load_model:
    def __init__(self):
        
        (self.angle, self.angular_speed, self.angular_acceleration) = (0, 0, 0)
        (self.x_position, self.x_speed, self.x_acceleration) = (400, 0, 0)
        (self.y_position, self.y_speed, self.y_acceleration) = (400, 0, 0)
        self.target_counter = 0
        self.dead = False
        self.respawn_timer = 3

        self.name = "SAC"
        model_path = "training_environment/tmp/rl_model_v2_1900000_steps.zip"
        model_path = os.path.join(os.path.dirname(__file__), model_path)
        self.path = model_path

        self.action_value = SAC.load(self.path)

    def act(self, obs):
        action, _ = self.action_value.predict(obs)
        (action0, action1, rotation_action) = (action[0], action[1], action[2])
        return (action0, action1, rotation_action)
