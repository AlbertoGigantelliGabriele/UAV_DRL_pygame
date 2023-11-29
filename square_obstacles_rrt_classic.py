import threading
import pygame
import random
import math

# Initialize Pygame and constants
pygame.init()
WINDOW_SIZE = 800, 800
WHITE, GREEN, BLUE, RED, BLACK = (255, 255, 255), (0, 255, 0), (0, 0, 255), (255, 0, 0), (0, 0, 0)
NODE_RADIUS, GOAL_RADIUS, MAX_ITERATIONS = 10, 45, 1500

"""class Node:
    def __init__(self, x, y): self.x, self.y, self.parent = x, y, None
    def get_pos(self): return self.x, self.y"""

"""def is_collision_free(node, obstacles, radius=NODE_RADIUS):
    for obstacle in obstacles:
        if distance(node, Node(obstacle[0], obstacle[1])) < radius:
            return False
    return True"""

class Node:
    def __init__(self, x, y):
        self.x, self.y, self.parent, self.cost = x, y, None, 0  # Aggiunto l'attributo di costo
    def get_pos(self):
        return self.x, self.y

def is_collision_free(node, obstacles, radius=GOAL_RADIUS):
    for obstacle in obstacles:
        # Distanza dal centro del cerchio all'angolo più vicino del rettangolo
        dx = max(obstacle.left - node.x, 0, node.x - obstacle.right)
        dy = max(obstacle.top - node.y, 0, node.y - obstacle.bottom)
        
        # Se la distanza è minore del raggio, c'è una collisione
        if (dx*dx + dy*dy) < radius*radius:
            return False
    return True

def get_path(end_node):
    path = []
    current_node = end_node
    while current_node is not None:
        path.append(current_node)
        current_node = current_node.parent
    return path[::-1]  # Inverti l'ordine per iniziare dal nodo di partenza

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
        if is_collision_free(start, obstacles, radius=GOAL_RADIUS):
            break

    while True:
        end = Node(random.randint(50, 750), random.randint(50, 750))
        if is_collision_free(end, obstacles, radius=GOAL_RADIUS):
            break

    return obstacles, start, end

def rrt(start, end, obstacles, shared_dict):
    nodes = [start]
    end_node = None

    while True:
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
        if not is_collision_free(new_node, obstacles, radius=GOAL_RADIUS):
            continue  # Skip the rest of this loop iteration

        # Add the node to the list
        if shared_dict is None:
            nodes.append(new_node)
            shared_dict['nodes'] = nodes
        else: 
            shared_dict['nodes'].append(new_node)

        # Check for goal
        if distance(new_node, end) < GOAL_RADIUS:
            end.parent = new_node
            shared_dict['nodes'].append(new_node)
            end_node = end  # Update end_node to indicate the goal was reached
            continue  # Exit the loop
            
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

        if not is_collision_free(new_node, obstacles, radius=GOAL_RADIUS):
            continue  # Skip the rest of this loop iteration
        
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

def draw_rrt(win, nodes, color = RED, dim = 2):
    for node in nodes:
        if node.parent:
            pygame.draw.line(win, color, (node.parent.x, node.parent.y), (node.x, node.y), dim)

def main():

    win = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("RRT Path Planning")

    clock = pygame.time.Clock()
    obstacles, start, end = setup_environment(6) 

    shared_dict = {'nodes': [], 'end_node': None}
    rrt_thread = threading.Thread(target=rrt_star, args=(start, end, obstacles, shared_dict))
    rrt_thread.daemon = True
    rrt_thread.start()

    dragging_start = False  
    dragging_end = False 
    
    nodes = None  
    path = None

    run = True
    while run:
        
        win.fill(WHITE) # reset the windows every time
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                try: 
                    rrt_thread._stop()
                except AssertionError:
                    exit(0)
                    
                # rrt_thread.join()
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                x, y = event.pos
                if distance(Node(x, y), start) < GOAL_RADIUS:
                    dragging_start = True
                elif distance(Node(x, y), end) < GOAL_RADIUS:
                    dragging_end = True

            elif event.type == pygame.MOUSEBUTTONUP:
                if dragging_start:  # Se uno dei due nodi è stato trascinato, riavvia RRT
                    try:
                        rrt_thread._stop()
                    except AssertionError:
                        pass                    
                        
                    shared_dict = {'nodes': [], 'end_node': None}
                    rrt_thread = threading.Thread(target=rrt, args=(start, end, obstacles, shared_dict))
                    rrt_thread.daemon = True
                    rrt_thread.start()

                dragging_start = False
                dragging_end = False
                    
            elif event.type == pygame.MOUSEMOTION:
                x, y = event.pos
                if dragging_start:
                    start.x, start.y = x, y
                elif dragging_end:
                    end.x, end.y = x, y

        (nodes, end_node) = (shared_dict['nodes'], shared_dict['end_node'])
        
        if nodes:
            draw_rrt(win, nodes, RED, 2)
            
        if end_node is not None:
            path = get_path(end_node)
            draw_rrt(win, path, BLACK, 5)

        # Draw obstacles and start/end circles
        for obstacle in obstacles:
            pygame.draw.rect(win, BLACK, obstacle)
        pygame.draw.circle(win, GREEN, (start.x, start.y), GOAL_RADIUS)  
        pygame.draw.circle(win, BLUE, (end.x, end.y), GOAL_RADIUS)  
        pygame.display.update()
        clock.tick(90)

if __name__ == "__main__":
    main()
