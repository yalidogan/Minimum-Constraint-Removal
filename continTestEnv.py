import numpy as np
import matplotlib.pyplot as plt
import random
import math
import matplotlib
matplotlib.use('TkAgg') #For pycharm to run matplotlib

#This is the test environment for a continuous grid
#The prev grid was a discrete one
#Here the planner uses an RRT implementation
#We can see the expanding tree and the path the robot can take to reach the goal
#The next document will be the MCR extension for this

# =========================================================
# 1. Generate Random Grid Map
# =========================================================
def generate_random_grid(width, height, obstacle_density=0.2, seed=None):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    grid = np.zeros((height, width), dtype=int)
    num_obstacles = int(obstacle_density * width * height)
    obstacle_indices = random.sample(range(width * height), num_obstacles)
    for idx in obstacle_indices:
        x, y = idx % width, idx // width
        grid[y][x] = 1

    free_cells = list(zip(*np.where(grid == 0)))
    start = random.choice(free_cells)
    goal = random.choice(free_cells)
    while goal == start:
        goal = random.choice(free_cells)

    return grid, start, goal

# =========================================================
# 2. RRT Implementation
# =========================================================
class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

def distance(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

def is_collision(x, y, grid):
    h, w = grid.shape
    if x < 0 or y < 0 or x >= w or y >= h:
        return True
    return grid[int(y)][int(x)] == 1

def get_nearest_node(nodes, random_node):
    return min(nodes, key=lambda node: distance(node, random_node))

def steer(from_node, to_node, step_size=2):
    theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = from_node.x + step_size * math.cos(theta)
    new_y = from_node.y + step_size * math.sin(theta)
    return Node(new_x, new_y, from_node)

def rrt_planner(grid, start, goal, max_iter=1000, step_size=2):
    start_node = Node(*start)
    goal_node = Node(*goal)
    nodes = [start_node]

    for i in range(max_iter):
        rand_x = random.uniform(0, grid.shape[1])
        rand_y = random.uniform(0, grid.shape[0])
        random_node = Node(rand_x, rand_y)

        nearest = get_nearest_node(nodes, random_node)
        new_node = steer(nearest, random_node, step_size)

        if not is_collision(new_node.x, new_node.y, grid):
            nodes.append(new_node)

            # Check if goal is reached
            if distance(new_node, goal_node) < step_size * 1.5:
                goal_node.parent = new_node
                nodes.append(goal_node)
                print(f"Goal reached in {i} iterations!")
                return nodes, goal_node

    print("Goal not reached.")
    return nodes, None

def extract_path(goal_node):
    path = []
    node = goal_node
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

# =========================================================
# 3. Visualization
# =========================================================
def visualize(grid, nodes, path, start, goal):
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap="gray_r", origin="upper")
    plt.scatter(start[0], start[1], c="green", s=80, label="Start")
    plt.scatter(goal[0], goal[1], c="red", s=80, label="Goal")

    # Draw tree edges
    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color="blue", linewidth=0.5)

    # Draw path
    if path:
        px, py = zip(*path)
        plt.plot(px, py, color="orange", linewidth=2, label="Path")

    plt.legend()
    plt.title("RRT Sampling-Based Planner Test Environment")
    plt.show()

# =========================================================
# 4. Run the test
# =========================================================
if __name__ == "__main__":
    grid, start, goal = generate_random_grid(50, 50, obstacle_density=0.25, seed=42)
    print("Start:", start, "Goal:", goal)

    nodes, goal_node = rrt_planner(grid, start, goal, max_iter=2000, step_size=2)

    path = extract_path(goal_node) if goal_node else []
    visualize(grid, nodes, path, start, goal)
