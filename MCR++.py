import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
import heapq
import itertools
import random
from math import sqrt


# ==============================
# ENVIRONMENT GENERATION
# ==============================
def generate_random_grid(size=60, obstacle_density=0.25, soft_ratio=0.4, seed=None):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
    grid = np.zeros((size, size), dtype=int)
    num_obstacles = int(obstacle_density * size * size)
    for _ in range(num_obstacles):
        x, y = np.random.randint(0, size, 2)
        grid[x, y] = 1 if random.random() > soft_ratio else 2
    return grid


def bresenham_line(x1, y1, x2, y2):
    """Generate grid cells on a line (for collision checking)."""
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    points = []
    dx, dy = abs(x2 - x1), abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy
    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    return points


def check_edge(from_pos, to_pos, grid):
    """Return if hard collision and number of soft cells crossed."""
    line = bresenham_line(*from_pos, *to_pos)
    soft_cost = 0
    for (x, y) in line:
        if x < 0 or y < 0 or x >= grid.shape[0] or y >= grid.shape[1]:
            return True, 0, []  # outside grid
        if grid[x, y] == 1:  # hard constraint
            return True, 0, []
        if grid[x, y] == 2:  # soft constraint
            soft_cost += 1
    return False, soft_cost, line


# ==============================
# NODE + PLANNER
# ==============================
class Node:
    def __init__(self, pos, parent=None, cost=0, soft_cost=0, violated=None):
        self.pos = pos
        self.parent = parent
        self.cost = cost
        self.soft_cost = soft_cost
        self.violated = violated or set()

    def total_cost(self, penalty):
        return self.cost + penalty * self.soft_cost


class RRTMCR:
    def __init__(self, grid, start, goal, step_size=3, max_iters=5000, penalty=5, goal_bias=0.1):
        self.grid = grid
        self.start = Node(start)
        self.goal = Node(goal)
        self.step_size = step_size
        self.max_iters = max_iters
        self.penalty = penalty
        self.goal_bias = goal_bias
        self.nodes = [self.start]

    def get_random_point(self):
        if random.random() < self.goal_bias:
            return self.goal.pos
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
        counter = itertools.count()
        heap = []
        heapq.heappush(heap, (self.start.total_cost(self.penalty), next(counter), self.start))

        for _ in range(self.max_iters):
            rand_point = self.get_random_point()
            nearest_node = self.nearest(rand_point)
            new_pos = self.steer(nearest_node, rand_point)

            collision, soft_cost, crossed = check_edge(nearest_node.pos, new_pos, self.grid)
            if collision:
                continue

            dist_cost = sqrt((new_pos[0]-nearest_node.pos[0])**2 + (new_pos[1]-nearest_node.pos[1])**2)
            new_soft_cost = nearest_node.soft_cost + soft_cost
            new_violations = nearest_node.violated | {(x, y) for (x, y) in crossed if self.grid[x, y] == 2}

            new_node = Node(new_pos, nearest_node, nearest_node.cost + dist_cost, new_soft_cost, new_violations)
            self.nodes.append(new_node)
            heapq.heappush(heap, (new_node.total_cost(self.penalty), next(counter), new_node))

            # Check if goal is reached
            if sqrt((new_pos[0]-self.goal.pos[0])**2 + (new_pos[1]-self.goal.pos[1])**2) < self.step_size:
                path = self.reconstruct_path(new_node)
                return {
                    "path": path,
                    "total_cost": new_node.total_cost(self.penalty),
                    "path_length": new_node.cost,
                    "soft_violations": len(new_node.violated),
                    "removed_constraints": new_node.violated
                }
        return None

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append(node.pos)
            node = node.parent
        return path[::-1]


# ==============================
# RUN TEST
# ==============================
if __name__ == "__main__":
    grid = generate_random_grid(size=60, obstacle_density=0.25, soft_ratio=0.4, seed=0)
    start = (5, 5)
    goal = (55, 55)

    planner = RRTMCR(grid, start, goal, step_size=3, max_iters=6000, penalty=10, goal_bias=0.15)
    result = planner.plan()

    plt.figure(figsize=(8, 8))
    plt.imshow(grid.T, origin="lower", cmap="gray_r")

    plt.scatter(*start, c='green', s=80, label="Start")
    plt.scatter(*goal, c='blue', s=80, label="Goal")

    if result:
        path = result["path"]
        xs, ys = zip(*path)
        plt.plot(xs, ys, 'r-', linewidth=2, label="Path")

        for (x, y) in result["removed_constraints"]:
            plt.scatter(x, y, c='orange', s=10)

        print("\n Path found!")
        print(f"  Total Cost: {result['total_cost']:.2f}")
        print(f"  Path Length: {result['path_length']:.2f}")
        print(f"  Soft Constraints Removed: {result['soft_violations']}")
    else:
        print("\n No path found")

    plt.legend()
    plt.show()
