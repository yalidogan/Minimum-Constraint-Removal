import numpy as np
import matplotlib.pyplot as plt
import heapq
import random
from math import sqrt
import matplotlib
import itertools
matplotlib.use('TkAgg')

# ========================
#  Environment Definition
# ========================
def generate_random_grid(size=50, obstacle_density=0.2, soft_ratio=0.5):
    #Randomly generate grid with removable and non-removable constraints

    grid = np.zeros((size, size), dtype=int)
    num_obstacles = int(obstacle_density * size * size)

    for _ in range(num_obstacles):
        x, y = np.random.randint(0, size, 2)
        grid[x, y] = 1 if random.random() > soft_ratio else 2

    return grid


def is_collision(p1, p2, grid):
    #check for collision
    x1, y1 = p1
    x2, y2 = p2
    n_points = int(sqrt((x2 - x1)**2 + (y2 - y1)**2))
    for i in range(n_points + 1):
        x = int(x1 + (x2 - x1) * i / max(n_points, 1))
        y = int(y1 + (y2 - y1) * i / max(n_points, 1))
        if x < 0 or y < 0 or x >= grid.shape[0] or y >= grid.shape[1]:
            return True, 0
        if grid[x, y] == 1:  # hard obstacle
            return True, 0
    return False, 0


def soft_violation_cost(p1, p2, grid):
    #Calculate how many soft obstacles are crossed
    x1, y1 = p1
    x2, y2 = p2
    n_points = int(sqrt((x2 - x1)**2 + (y2 - y1)**2))
    cost = 0
    for i in range(n_points + 1):
        x = int(x1 + (x2 - x1) * i / max(n_points, 1))
        y = int(y1 + (y2 - y1) * i / max(n_points, 1))
        if grid[x, y] == 2:
            cost += 1
    return cost


# ========================
#  Node & Planner Classes
# ========================
class Node:
    def __init__(self, pos, parent=None, cost=0, soft_cost=0):
        self.pos = pos
        self.parent = parent
        self.cost = cost
        self.soft_cost = soft_cost

    def total_cost(self, penalty=5):
        return self.cost + penalty * self.soft_cost


class RRTMCR:
    def __init__(self, grid, start, goal, step_size=3, max_iters=5000, penalty=5):
        self.grid = grid
        self.start = Node(start)
        self.goal = Node(goal)
        self.step_size = step_size
        self.max_iters = max_iters
        self.penalty = penalty
        self.nodes = [self.start]

    def get_random_point(self):
        return (random.randint(0, self.grid.shape[0] - 1),
                random.randint(0, self.grid.shape[1] - 1))

    def nearest(self, point):
        return min(self.nodes, key=lambda n: sqrt((n.pos[0]-point[0])**2 + (n.pos[1]-point[1])**2))

    def steer(self, from_node, to_point):
        x1, y1 = from_node.pos
        x2, y2 = to_point
        dx, dy = x2 - x1, y2 - y1
        dist = sqrt(dx**2 + dy**2)
        if dist == 0:
            return from_node.pos
        scale = min(self.step_size / dist, 1)
        new_pos = (int(x1 + dx * scale), int(y1 + dy * scale))
        return new_pos

    def plan(self):
        heap = []
        counter = itertools.count()  # unique sequence count
        heapq.heappush(heap, (self.start.total_cost(self.penalty), next(counter), self.start))

        for _ in range(self.max_iters):
            rand_point = self.get_random_point()
            nearest_node = self.nearest(rand_point)
            new_pos = self.steer(nearest_node, rand_point)

            collision, _ = is_collision(nearest_node.pos, new_pos, self.grid)
            if collision:
                continue  # skip hard obstacle paths

            soft_cost = soft_violation_cost(nearest_node.pos, new_pos, self.grid)
            cost = nearest_node.cost + sqrt((new_pos[0]-nearest_node.pos[0])**2 + (new_pos[1]-nearest_node.pos[1])**2)
            new_node = Node(new_pos, nearest_node, cost, nearest_node.soft_cost + soft_cost)
            self.nodes.append(new_node)

            # use counter to break ties
            heapq.heappush(heap, (new_node.total_cost(self.penalty), next(counter), new_node))

            if sqrt((new_pos[0]-self.goal.pos[0])**2 + (new_pos[1]-self.goal.pos[1])**2) < 5:
                return self.reconstruct_path(new_node)
        return None

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append(node.pos)
            node = node.parent
        return path[::-1]


# ========================
#  Run & Visualize
# ========================

if __name__ == "__main__":
    grid = generate_random_grid(size=60, obstacle_density=0.25, soft_ratio=0.4)
    start = (5, 5)
    goal = (55, 55)

    planner = RRTMCR(grid, start, goal, step_size=3, max_iters=4000, penalty=10)
    path = planner.plan()

    plt.imshow(grid.T, origin="lower", cmap="gray_r")
    if path:
        xs, ys = zip(*path)
        plt.plot(xs, ys, 'r-', linewidth=2)
        plt.scatter(*start, c='green', s=80, label="Start")
        plt.scatter(*goal, c='blue', s=80, label="Goal")
    plt.legend()
    plt.show()
