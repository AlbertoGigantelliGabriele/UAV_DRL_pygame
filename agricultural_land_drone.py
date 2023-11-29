import os
os.environ['KMP_DUPLICATE_LIB_OK']='TRUE'

import random
from random import randrange
from math import sin, cos, pi, sqrt
import math
import sys
import numpy as np
import pygame
from pygame.locals import *
from model_actions import Load_model
import cv2
from rrtplanner import RRTStarInformed

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

def setup_environment(sub_img, x, y):
    
    img_width, img_height = sub_img.get_size()
    pixel_array = pygame.surfarray.array3d(sub_img)

    obstacles = []
    for y in range(img_height):
        for x in range(img_width):
            r, g, b = pixel_array[x][y]
            if g > 200 and r < 50 and b < 50:
                obstacles.append((x, y))

    red_centroids = calculate_red_centroids(sub_img)
    targets = []
    for centroid in red_centroids:
        # Invertiamo (y, x) a (x, y) per adattarci a Pygame.
        targets.append((centroid[1], centroid[0]))

    return obstacles, targets

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

def calculate_red_centroids(sub_img):
    red_centroids = []
    img_array = pygame.surfarray.array3d(sub_img)
    img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
    img_rgb = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)
    lower_red = np.array([100, 0, 0])
    upper_red = np.array([255, 50, 50])
    mask = cv2.inRange(img_rgb, lower_red, upper_red)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            red_centroids.append((cX, cY))

    return red_centroids

WINDOW_SIZE = SCREEN_W, SCREEN_H = 800, 800
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)

pygame.init()

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

def draw_vectors(screen, agent, targets):
    """
    Disegna i vettori per la velocità, la spinta, la direzione effettiva, e l'angolo di orientamento.
    """
    # Definizione dei colori chiari
    color_velocity = (255, 0, 0)  
    color_thrust = (0, 255, 0)    
    color_movement = (0, 100, 255)  
    color_orientation = (255, 255, 0)  
    color_target = (0, 255, 255)      

    # Assicurati che ci siano ancora target
    if agent.target_counter < len(targets):
        target = targets[agent.target_counter]

        # Vettore della velocità (proporzionale alla velocità)
        scale_factor = 5  # Puoi modificare questo fattore per cambiare la scala del vettore
        velocity_vector_end = (agent.x_position + agent.x_speed * scale_factor, agent.y_position + agent.y_speed * scale_factor)

        # Calcolo dell'angolo di spinta e della direzione effettiva di movimento
        thrust_angle = agent.angle
        move_angle = math.atan2(agent.y_speed, agent.x_speed) if agent.x_speed != 0 or agent.y_speed != 0 else 0

        # Calcolo delle coordinate finali per i vettori
        thrust_x = agent.x_position + 30 * cos(thrust_angle)
        thrust_y = agent.y_position + 30 * sin(thrust_angle)
        move_x = agent.x_position + 30 * cos(move_angle)
        move_y = agent.y_position + 30 * sin(move_angle)
        orientation_x = agent.x_position + 30 * cos(agent.angle)
        orientation_y = agent.y_position + 30 * sin(agent.angle)

        # Disegno dei vettori con i colori definiti
        pygame.draw.line(screen, color_velocity, (agent.x_position, agent.y_position), velocity_vector_end, 2)
        pygame.draw.line(screen, color_thrust, (agent.x_position, agent.y_position), (thrust_x, thrust_y), 2)
        pygame.draw.line(screen, color_movement, (agent.x_position, agent.y_position), (move_x, move_y), 2)
        pygame.draw.line(screen, color_orientation, (agent.x_position, agent.y_position), (orientation_x, orientation_y), 2)
        pygame.draw.line(screen, color_target, (agent.x_position, agent.y_position), target, 2)

    # Loading fonts
    pygame.font.init()
    
    # Disegno della legenda
    font = pygame.font.SysFont('Arial', 15)
    screen.blit(font.render('Velocità: Azzurro chiaro', True, color_velocity), (10, SCREEN_H - 90))
    screen.blit(font.render('Spinta: Verde chiaro', True, color_thrust), (10, SCREEN_H - 70))
    screen.blit(font.render('Movimento: Rosa chiaro', True, color_movement), (10, SCREEN_H - 50))
    screen.blit(font.render('Orientamento: Blu chiaro', True, color_orientation), (10, SCREEN_H - 30))
    screen.blit(font.render('Target: Giallo chiaro', True, color_target), (10, SCREEN_H - 10))

def environment():
    FPS = 60
    WIDTH = 800
    HEIGHT = 800
    IMG_SIZE = 350
    
    thruster_amplitude = 0.04
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

    time = 0
    step = 0
    time_limit = 100
    respawn_timer_max = 1

    def handle_event(event):
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type in [pygame.KEYDOWN, pygame.KEYUP]:
            key_map = {pygame.K_LEFT: 'left', pygame.K_RIGHT: 'right', pygame.K_UP: 'up', pygame.K_DOWN: 'down'}
            if event.key in key_map:
                move[key_map[event.key]] = event.type == pygame.KEYDOWN

    agent = Load_model()

    #obstacles, start, end = setup_environment(6)  # Ho aggiunto un numero massimo di ostacoli come esempio
    ( agent.angle, agent.angular_speed, agent.angular_acceleration, ) = (0, 0, 0,)
    ( agent.x_position, agent.x_speed, agent.x_acceleration ) = (10, 0, 0,)
    ( agent.y_position, agent.y_speed, agent.y_acceleration ) = (10, 0, 0,)
    
    x, y = 0, 0
    img = pygame.image.load(correct_path(os.path.join("images/field/MicrosoftTeams-image.png")))
    img_width, img_height = img.get_size()
    screen = pygame.display.set_mode((WINDOW_SIZE))
    x, y = 0, 0
    
    move = {'left': False, 'right': False, 'up': False, 'down': False}

    draw_rrt_ = False
    was_moving = False

    while True:
    
        # CHANGE IMAGE POSITION
        [handle_event(event) for event in pygame.event.get()]

        dx, dy = (move['right'] - move['left']) * 10, (move['down'] - move['up']) * 10
        x, y = [max(0, min(coord, img_dim - IMG_SIZE - 1)) for coord, img_dim in zip([x + dx, y + dy], [img_width, img_height])]

        sub_img = img.subsurface((x, y, IMG_SIZE, IMG_SIZE))
        scaled_img = pygame.transform.scale(sub_img, (WINDOW_SIZE))
        
        screen.blit(scaled_img, (0, 0))
    
        time += 1 / 60
        step += 1

        if not any(move.values()) and was_moving:

            obstacles, targets = setup_environment(scaled_img, x, y)
            og = np.zeros((800, 800))

            #for x_tmp, y_tmp in obstacles:
                #og[x_tmp][y_tmp] = 1

            n = 1200
            r_rewire = 80
            r_goal = 12 

            rrts = RRTStarInformed(og, n, r_rewire, r_goal)
            start = (10, 10) # random_point_og(og)

            start_node = Node(start[0], start[1])
            sorted_targets = []
            current_start = start_node

            while targets:
                nearest_target = min(targets, key=lambda target: distance(current_start, Node(*target)))
                targets.remove(nearest_target)  
                sorted_targets.append(nearest_target)  
                current_start = Node(*nearest_target)

            targets = sorted_targets

            draw_rrt_ = True

        was_moving = any(move.values())

        if draw_rrt_:
            if agent.dead == False:
                
                # Initialize accelerations
                agent.x_acceleration = 0
                agent.y_acceleration = 0
                agent.angular_acceleration = 0

                # draw_vectors(screen, agent, targets)
                
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

        if time > time_limit:
            break

        pygame.draw.rect(screen, (255, 255, 255), (x / img_width * 800, 780, (350 / img_width) * 800, 20))
        pygame.draw.rect(screen, (255, 255, 255), (780, y / img_height * 800, 20, (350 / img_height) * 800))
        pygame.display.update()
        FramePerSec.tick(FPS)

environment()
