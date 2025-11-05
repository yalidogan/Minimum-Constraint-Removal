import numpy as np
import matplotlib.pyplot as plt
import random
import math
import heapq
import matplotlib
matplotlib.use('TkAgg') #For pycharm to run matplotlib

#Extension of the previous test environment
#Uses RRT and find the min path with MCR with a removal cost
#Has a cap of max cost to not breach in order to stay optimal

# =========================================================
# 1. Generate Random Grid Map with Soft Constraints
# =========================================================
def generate_random_grid(width, height, obstacle_density=0.2, seed=None):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    # Grid: 0 = free space, 1 = obstacle
    grid = np.zeros((height, width), dtype=int)
    num_obstacles = int(obstacle_density * width * height)
    obstacle_indices = random.sample(range(width * height), num_obstacles)
    for idx in obstacle_indices:
        x, y = idx % width, idx // width
        grid[y][x] = 1

    # Assign random "removal costs" to obstacles
    costs = np.zeros_like(grid, dtype=float)
    for y in range(height):
        for x in range(width):
            if grid[y][x] == 1:
                costs[y][x] = random.uniform(1.0, 5.0)  # cost to remove

    free_cells = list(zip(*np.where(grid == 0)))
    start = random.choice(free_cells)
    goal = random.choice(free_cells)
    while goal == start:
        goal = random.choice(free_cells)

    return grid, costs, start, goal

# =========================================================
# 2. Basic RRT Implementation
# =========================================================
class Node:
    def __init__(self, x, y, parent=None, cost=0.0, removed_constraints=set()):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost  # path cost
        self.removed_constraints = removed_constraints  # set of (x,y) obstacles removed

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

# =========================================================
# 3. MCR-RRT: Minimum Constraint Removal Extension
# =========================================================
def mcr_rrt(grid, costs, start, goal, max_iter=2000, step_size=2, max_removal_cost=10.0):
    start_node = Node(*start)
    goal_node = Node(*goal)
    nodes = [start_node]
    best_goal = None

    for i in range(max_iter):
        # Random sampling
        rand_x = random.uniform(0, grid.shape[1])
        rand_y = random.uniform(0, grid.shape[0])
        random_node = Node(rand_x, rand_y)

        nearest = get_nearest_node(nodes, random_node)
        new_node = steer(nearest, random_node, step_size)

        x, y = int(new_node.x), int(new_node.y)
        new_removed = set(nearest.removed_constraints)
        new_cost = nearest.cost

        # Handle obstacle encounter (potential constraint removal)
        if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]:
            if grid[y][x] == 1:  # obstacle
                removal_cost = costs[y][x]
                if new_cost + removal_cost > max_removal_cost:
                    continue  # skip, too expensive to remove
                new_removed.add((x, y))
                new_cost += removal_cost

        new_node.removed_constraints = new_removed
        new_node.cost = new_cost
        nodes.append(new_node)

        # Check goal proximity
        if distance(new_node, goal_node) < step_size * 1.5:
            goal_node.parent = new_node
            goal_node.cost = new_node.cost
            goal_node.removed_constraints = new_node.removed_constraints
            best_goal = goal_node
            print(f"Goal reached at iteration {i}, removal cost={goal_node.cost:.2f}")
            break

    if not best_goal:
        print("Goal not reached.")
    else:
        print(f"Total removed constraints: {len(best_goal.removed_constraints)}")

    return nodes, best_goal

def extract_path(goal_node):
    path = []
    node = goal_node
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

# =========================================================
# 4. Visualization
# =========================================================
def visualize(grid, costs, nodes, path, start, goal, removed):
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap="gray_r", origin="upper")
    plt.scatter(start[0], start[1], c="green", s=80, label="Start")
    plt.scatter(goal[0], goal[1], c="red", s=80, label="Goal")

    # Draw exploration tree
    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color="blue", linewidth=0.4)

    # Draw final path
    if path:
        px, py = zip(*path)
        plt.plot(px, py, color="orange", linewidth=2, label="Path")

    # Highlight removed constraints
    if removed:
        rx, ry = zip(*removed)
        plt.scatter(rx, ry, c="purple", marker="x", s=60, label="Removed Obstacles")

    plt.legend()
    plt.title("Minimum Constraint Removal RRT")
    plt.show()

# =========================================================
# 5. Run Test Environment
# =========================================================
if __name__ == "__main__":
    grid, costs, start, goal = generate_random_grid(40, 40, obstacle_density=0.3, seed=1)
    print("Start:", start, "Goal:", goal)

    nodes, goal_node = mcr_rrt(grid, costs, start, goal, max_iter=1500, step_size=2, max_removal_cost=10.0)
    path = extract_path(goal_node) if goal_node else []
    removed = goal_node.removed_constraints if goal_node else set()

    visualize(grid, costs, nodes, path, start, goal, removed)
