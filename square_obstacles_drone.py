import os
import random
from random import randrange
from math import sin, cos, pi, sqrt
import math
import sys
import numpy as np
import pygame
from pygame.locals import *
from model_actions import Load_model

def correct_path(current_path):
    """
    This function is used to get the correct path to the assets folder
    """
    return os.path.join(os.path.dirname(__file__), current_path)

def calculate_angle_to_up(a):
    return a / 180 * pi

def calculate_velocity(xd, yd):
    return sqrt(xd**2 + yd**2)

def calculate_angle_velocity(ad):
    return ad

def calculate_distance_to_target(x, y, xt, yt):
    return sqrt((xt - x) ** 2 + (yt - y) ** 2) 

def calculate_angle_to_target(x, y, xt, yt):
    return np.arctan2(yt - y, xt - x)

def calculate_angle_target_and_velocity(x, y, xd, yd, xt, yt):
    return np.arctan2(yt - y, xt - x) - np.arctan2(yd, xd)

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
    
    def get_pos(self):
        return (self.x, self.y)

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def setup_environment(n):
    obstacles = []
    max_attempts = 100
    min_distance_between_centers = 90 + 100  # 90 è la somma delle metà dei lati dei due rettangoli, 100 è la distanza voluta

    for _ in range(n):
        attempts = 0
        while attempts < max_attempts:
            new_obstacle = pygame.Rect(random.randint(50, 700), random.randint(50, 700), 90, 90)
            
            # Calcola il centro del nuovo ostacolo
            new_center = Node(new_obstacle.x + new_obstacle.width // 2, new_obstacle.y + new_obstacle.height // 2)
            
            if all(distance(Node(rect.x + rect.width // 2, rect.y + rect.height // 2), new_center) >= min_distance_between_centers for rect in obstacles):
                obstacles.append(new_obstacle)
                break
            attempts += 1

    # Posiziona i nodi di partenza e di arrivo in aree libere
    while True:
        start = Node(random.randint(50, 750), random.randint(50, 750))
        if is_collision_free(start, obstacles, radius=45):
            break

    while True:
        end = Node(random.randint(50, 750), random.randint(50, 750))
        if is_collision_free(end, obstacles, radius=45):
            break

    return obstacles, start, end

def is_collision_free(node, obstacles, radius=45):
    for obstacle in obstacles:
        # Distanza dal centro del cerchio all'angolo più vicino del rettangolo
        dx = max(obstacle.left - node.x, 0, node.x - obstacle.right)
        dy = max(obstacle.top - node.y, 0, node.y - obstacle.bottom)
        
        # Se la distanza è minore del raggio, c'è una collisione
        if (dx*dx + dy*dy) < radius*radius:
            return False
    return True

def rrt(start, end, obstacles):
    nodes = [start]
    max_iterations = 5000
    goal_radius = 15
    winning_path = []  # Lista per il percorso vincente
    
    for _ in range(max_iterations):
        # Random sampling
        rand_x = random.randint(0, 800)
        rand_y = random.randint(0, 800)
        rand_node = Node(rand_x, rand_y)
        
        # Find the nearest node
        nearest_node = min(nodes, key=lambda node: distance(node, rand_node))
        
        # Create a new node
        theta = math.atan2(rand_node.y - nearest_node.y, rand_node.x - nearest_node.x)
        new_node = Node(nearest_node.x + 10 * math.cos(theta), nearest_node.y + 10 * math.sin(theta))
        new_node.parent = nearest_node
        
        # Check for collision
        if not is_collision_free(new_node, obstacles, radius=45):
            continue
        
        # Add the node to the list
        nodes.append(new_node)
        
        # Check for goal
        if distance(new_node, end) < goal_radius:
            end.parent = new_node
            nodes.append(end)
            
            # Riempi la lista del percorso vincente
            current_node = end
            while current_node is not None:
                winning_path.append(current_node)
                current_node = current_node.parent
            
            return winning_path  # Restituisci il percorso vincente
    
    return None

def draw_obstacles(screen, obstacles):
    for obstacle in obstacles:
            pygame.draw.rect(screen, (255, 0, 0), obstacle)

GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)

def draw_path_from_coordinates(screen, targets):
    for x, y in targets:
        pygame.draw.circle(screen, YELLOW, (x, y), 5)

def draw_rrt(screen, end_node):
    current_node = end_node
    while current_node.parent is not None:
        pygame.draw.line(screen, BLUE, (current_node.parent.x, current_node.parent.y), (current_node.x, current_node.y), 2)
        
        # Crea una superficie trasparente
        # s = pygame.Surface((800, 800), pygame.SRCALPHA)
        
        # Disegna un cerchio semi-trasparente sulla superficie
        pygame.draw.circle(screen, (255, 0, 0, 128), (current_node.x, current_node.y), 45)
        
        # Blit la superficie trasparente sulla finestra principale
        #screen.blit(screen, (0, 0))
        current_node = current_node.parent
    
    if not current_node.parent:
        print('None')

def draw_points(screen, start, end):
    pygame.draw.circle(screen, GREEN, (start.x, start.y), 45)  # Raggio cambiato a 45 per avere un diametro di 90
    pygame.draw.circle(screen, BLUE, (end.x, end.y), 45)  # Raggio cambiato a 45 per avere un diametro di 90

def environment():

    # Constants
    FPS = 60
    WIDTH = 800
    HEIGHT = 800

    # Physics constants
    gravity = 0
    # Propeller force for UP and DOWN
    thruster_amplitude = 0.04
    # Propeller force for LEFT and RIGHT rotations
    diff_amplitude = 0.003
    # By default, thruster will apply angle force of thruster_mean
    thruster_mean = 0.04
    mass = 1
    # Length from center of mass to propeller
    arm = 25

    # Initialize Pygame, load sprites
    FramePerSec = pygame.time.Clock()

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))

    # Loading agent and target sprites
    agent_width = 80
    agent_animation_speed = 1
    agent_animation = []

    for i in range(1, 5):
        image = pygame.image.load(correct_path(os.path.join("images/drones/drone1/drone" + str(i) + ".png")))
        image.convert()
        agent_animation.append(
            pygame.transform.scale(image, (agent_width, int(agent_width * 1))))

    target_width = 30
    target_animation_speed = 0.1
    target_animation = []
    image = pygame.image.load(correct_path(os.path.join("images/target/simple-targets/target3.png")))
    image.convert()
    target_animation.append(pygame.transform.scale(image, (target_width, int(target_width * 1.73))))

    time = 0
    step = 0
    time_limit = 100
    respawn_timer_max = 3 # Three seconds before respawn like in games

    agent = Load_model()
       
    obstacles, start, end = setup_environment(6)  # Ho aggiunto un numero massimo di ostacoli come esempio
    ( agent.angle, agent.angular_speed, agent.angular_acceleration, ) = (0, 0, 0,)
    ( agent.x_position, agent.x_speed, agent.x_acceleration ) = (end.get_pos()[0], 0, 0,)
    ( agent.y_position, agent.y_speed, agent.y_acceleration ) = (end.get_pos()[1], 0, 0,)
    path = rrt(start, end, obstacles)

    if path:
        #print("Path found!")
        end_node = path[-1]
    else:
        #print("Path not found!")
        end_node = None

    targets = []
    for i in range(len(path)):
        targets.append(path[i].get_pos())

    while True:
    
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        
        pygame.event.get()
        
        # Display background
        screen.fill(0)

        time += 1 / 60
        step += 1

        draw_obstacles(screen, obstacles)
        if end_node:
                draw_path_from_coordinates(screen, targets)
        draw_points(screen, start, end)
        
        if agent.dead == False:
            
            # Initialize accelerations
            agent.x_acceleration = 0
            agent.y_acceleration = 0
            agent.angular_acceleration = 0

            # Calculate propeller force in function of input
            if agent.name == "SAC":

                angle_to_up = agent.angle / 180 * pi
                velocity = sqrt(agent.x_speed**2 + agent.y_speed**2)
                # angle_velocity = agent.angular_speed
                # distance_to_target = calculate_distance_to_target(agent.x_position, agent.y_position, targets[agent.target_counter][0], targets[agent.target_counter][1])
                angle_to_target = calculate_angle_to_target(agent.x_position, agent.y_position, targets[agent.target_counter][0], targets[agent.target_counter][1])
                angle_target_and_velocity = calculate_angle_target_and_velocity(agent.x_position, agent.y_position, agent.x_speed, agent.y_speed, targets[agent.target_counter][0], targets[agent.target_counter][1])

                distance_to_target = (sqrt((targets[agent.target_counter][0] - agent.x_position) ** 2 + (targets[agent.target_counter][1] - agent.y_position) ** 2 ) / 500)

                """angle_to_target = np.arctan2(
                    targets[agent.target_counter][1] - agent.y_position,
                    targets[agent.target_counter][0] - agent.x_position,
                )
                
                # Angle between the to_target vector and the velocity vector
                angle_target_and_velocity = np.arctan2(
                    targets[agent.target_counter][1] - agent.y_position,
                    targets[agent.target_counter][0] - agent.x_position,
                ) - np.arctan2(agent.y_speed, agent.x_speed)"""

                action0, action1, rotation_action = agent.act(
                    np.array([
                            angle_to_up,
                            velocity,
                            agent.angular_speed,
                            # angle_velocity,
                            distance_to_target,
                            angle_to_target,
                            angle_target_and_velocity,
                        ]).astype(np.float32)
                )

            # Calculate accelerations according to Newton's laws of motion
            agent.x_acceleration += math.cos(math.radians(agent.angle)) * action0 * thruster_amplitude - math.sin(math.radians(agent.angle)) * action1 * thruster_amplitude
            agent.y_acceleration += math.sin(math.radians(agent.angle)) * action0 * thruster_amplitude + math.cos(math.radians(agent.angle)) * action1 * thruster_amplitude

            # agent.angular_acceleration += (arm * (thruster_right - thruster_left) / mass)

            # Calculate speed
            agent.angular_speed = rotation_action # agent.angular_acceleration
            agent.angle += agent.angular_speed

            agent.x_speed += agent.x_acceleration
            agent.y_speed += agent.y_acceleration
            agent.angular_speed += agent.angular_acceleration

            # Calculate position
            agent.x_position += agent.x_speed
            agent.y_position += agent.y_speed

            # Calculate distance to target
            dist = calculate_distance_to_target(agent.x_position, agent.y_position, targets[agent.target_counter][0], targets[agent.target_counter][1])
            
            # If target reached, respawn target
            if dist < 50:
                agent.target_counter += 1

            # If to far, die and respawn after timer
            elif dist > 1000:
                agent.dead = True
                agent.respawn_timer = respawn_timer_max
        else:

            agent.respawn_timer -= 1 / 60
            # Respawn
            if agent.respawn_timer < 0:
                agent.dead = False
                ( agent.angle, agent.angular_speed, agent.angular_acceleration, ) = (0, 0, 0,)
                ( agent.x_position, agent.x_speed, agent.x_acceleration ) = (start.get_pos()[0], 0, 0,)
                ( agent.y_position, agent.y_speed, agent.y_acceleration ) = (start.get_pos()[1], 0, 0,)

        # Display target and agent
        pygame.draw.circle(screen, (255, 0, 255), (targets[agent.target_counter][0], targets[agent.target_counter][1]), 5)

        agent_sprite = agent_animation[ int(step * agent_animation_speed) % len(agent_animation)]
        agent_copy = pygame.transform.rotate(agent_sprite, agent.angle)
        screen.blit(
            agent_copy,
            (
                agent.x_position - int(agent_copy.get_width() / 2),
                agent.y_position - int(agent_copy.get_height() / 2),
            ),
        )

        # Ending conditions
        if time > time_limit:
            break

        pygame.display.update()
        FramePerSec.tick(FPS)

environment()
