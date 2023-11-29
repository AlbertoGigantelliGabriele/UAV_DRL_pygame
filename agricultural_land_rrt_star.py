import pygame
import sys
import random
import math
import cv2
import numpy as np
from rrtplanner import RRTStarInformed
import os

def correct_path(current_path):
    """
    This function is used to get the correct path to the assets folder
    """
    return os.path.join(os.path.dirname(__file__), current_path)

WINDOW_SIZE = SCREEN_W, SCREEN_H = 800, 800
IMG_SIZE = 350

pygame.init()
img = pygame.image.load(correct_path(os.path.join("images/field/MicrosoftTeams-image.png")))
img_width, img_height = img.get_size()
screen = pygame.display.set_mode((WINDOW_SIZE))
x, y = 0, 0
move = {'left': False, 'right': False, 'up': False, 'down': False}

class Node:
    def __init__(self, x, y):
        self.x, self.y, self.parent, self.cost = x, y, None, 0 
    def get_pos(self):
        return self.x, self.y

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

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

def setup_environment(sub_img, x, y):
    
    obstacles = []
    targets = []
    img_width, img_height = sub_img.get_size()
    pixel_array = pygame.surfarray.array3d(sub_img)

    for y in range(img_height):
        for x in range(img_width):
            r, g, b = pixel_array[x][y]
            if g > 200 and r < 50 and b < 50:
                obstacles.append((x, y))

    red_centroids = calculate_red_centroids(sub_img)
    for centroid in red_centroids:
        # Invertiamo (y, x) a (x, y) per adattarci a Pygame.
        targets.append((centroid[1], centroid[0]))

    while True:
        start = Node(random.randint(50, 750), random.randint(50, 750))
        if (start.x, start.y) not in obstacles:
            break

    return obstacles, targets, start

def rrt(start, end, obstacles, shared_dict):
    nodes = [start]
    end_node = None

    while True:
        
        # Random sampling
        rand_x = random.randint(0, 800)
        rand_y = random.randint(0, 800)
        rand_node = Node(rand_x, rand_y)
        
        nearest_node = min(nodes, key=lambda node: distance(node, rand_node))

        theta = math.atan2(rand_node.y - nearest_node.y, rand_node.x - nearest_node.x)
        new_node = Node(nearest_node.x + 10 * math.cos(theta), nearest_node.y + 10 * math.sin(theta))
        new_node.parent = nearest_node

        # Check for collision
        #if not is_collision_free(new_node, obstacles, radius=GOAL_RADIUS):
            #continue  # Skip the rest of this loop iteration

        if shared_dict is None:
            nodes.append(new_node)
            shared_dict['nodes'] = nodes
        else: 
            shared_dict['nodes'].append(new_node)

        if distance(new_node, end) < 20:
            end.parent = new_node
            shared_dict['nodes'].append(new_node)
            shared_dict['end_node'] = end
            end_node = end  # Update end_node to indicate the goal was reached
            break  # Exit the loop

        shared_dict['nodes'] = nodes
        shared_dict['end_node'] = end_node

def rrt_star(start, end, obstacles, shared_dict, radius=30, step_size=10):
    nodes = [start]
    end_node = None
    
    while True:
        rand_x = random.randint(0, 800)
        rand_y = random.randint(0, 800)
        rand_node = Node(rand_x, rand_y)

        nearest_node = min(nodes, key=lambda node: distance(node, rand_node))

        theta = math.atan2(rand_node.y - nearest_node.y, rand_node.x - nearest_node.x)
        new_node = Node(nearest_node.x + step_size * math.cos(theta), nearest_node.y + step_size * math.sin(theta))
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + distance(nearest_node, new_node)  # Aggiorna il costo

        # Se è libero da collisioni, aggiungilo.
        # (Da implementare: is_collision_free)
        # if is_collision_free(new_node, obstacles):
        nodes.append(new_node)

        # Re-wire: Cerca nodi vicini e vedi se passare attraverso il nuovo nodo riduce il costo.
        for node in nodes:
            if node == new_node or node == start:
                continue

            if distance(new_node, node) < radius:
                tentative_cost = new_node.cost + distance(new_node, node)  # Calcola il costo ipotetico

                if tentative_cost < node.cost:
                    node.parent = new_node
                    node.cost = tentative_cost

        if distance(new_node, end) < 20:
            end.parent = new_node
            end.cost = new_node.cost + distance(new_node, end)  # Aggiorna il costo
            end_node = end
            break

    shared_dict['nodes'] = nodes
    shared_dict['end_node'] = end_node

def handle_event(event):
    if event.type == pygame.QUIT:
        pygame.quit()
        sys.exit()
    if event.type in [pygame.KEYDOWN, pygame.KEYUP]:
        key_map = {pygame.K_LEFT: 'left', pygame.K_RIGHT: 'right', pygame.K_UP: 'up', pygame.K_DOWN: 'down'}
        if event.key in key_map:
            move[key_map[event.key]] = event.type == pygame.KEYDOWN

draw_rrt_ = False
was_moving = False
ignore_obstacles = False # Imposta a False se vuoi considerare gli ostacoli

while True:

    # CHANGE IMAGE POSITION
    [handle_event(event) for event in pygame.event.get()]

    dx, dy = (move['right'] - move['left']) * 10, (move['down'] - move['up']) * 10
    x, y = [max(0, min(coord, img_dim - IMG_SIZE - 1)) for coord, img_dim in zip([x + dx, y + dy], [img_width, img_height])]

    sub_img = img.subsurface((x, y, IMG_SIZE, IMG_SIZE))
    scaled_img = pygame.transform.scale(sub_img, (WINDOW_SIZE))

    screen.blit(scaled_img, (0, 0))

    # COMPUTE RRT
    if not any(move.values()) and was_moving:

        obstacles, targets, start = setup_environment(scaled_img, x, y)
        og = np.zeros((800, 800))

        if not ignore_obstacles:
            for x_tmp, y_tmp in obstacles:
                og[x_tmp][y_tmp] = 1

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

        current_start_node = Node(start[0], start[1])
        all_paths = []

        for target in sorted_targets:
            target_node = Node(*target)
            
            start_np = np.array(current_start_node.get_pos())
            goal_np = np.array(target_node.get_pos())
            T, gv = rrts.plan(start_np, goal_np)
            
            path = rrts.route2gv(T, gv)
            path_pts = rrts.vertices_as_ndarray(T, path)
            all_paths.append(path_pts)
            
            current_start_node = target_node
        
        
        """nearest_target = min(targets, key=lambda target: distance(start, Node(*target)))

        shared_dict = {'nodes': [], 'end_node': None}
        # rrt(start, Node(*nearest_target), obstacles, shared_dict)
        # rrt_star(start, Node(*nearest_target), obstacles, shared_dict)

        nodes = shared_dict['nodes']"""

        draw_rrt_ = True

    was_moving = any(move.values())

    # DRAW SECTION
    if draw_rrt_:

        for path_pts in all_paths:
            for line in path_pts:
                pygame.draw.line(screen, (255, 0, 255), tuple(line[0]), tuple(line[1]), 3)
        
        """node = shared_dict['end_node']
        while node.parent is not None:
            pygame.draw.line(screen, (255, 0, 0), (node.x, node.y), (node.parent.x, node.parent.y), 2)
            node = node.parent"""
        
        pygame.draw.circle(screen, (0, 0, 255), start, 10)
        pygame.draw.circle(screen, (255, 255, 0), sorted_targets[-1], 10)

        for target in sorted_targets: 
            pygame.draw.circle(screen, (255, 255, 0), target, 3)

    pygame.draw.rect(screen, (255, 255, 255), (x / img_width * 800, 780, (350 / img_width) * 800, 20))
    pygame.draw.rect(screen, (255, 255, 255), (780, y / img_height * 800, 20, (350 / img_height) * 800))

    pygame.display.flip()
