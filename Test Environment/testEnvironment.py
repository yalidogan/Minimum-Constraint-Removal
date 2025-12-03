import heapq
import matplotlib.pyplot as plt
import numpy as np
import random
import matplotlib
matplotlib.use('TkAgg')

#This is a test document utilizing a djikstra based search algorithm
#The code here creates a discrete grid of the given size
#Goal and the start are randomly assigned
#The algorithm finds the path from the start to goal
#Gives the min number of constr to remove from the shortest path


def generate_random_grid(width, height, obstacle_density=0.2, seed=None):

    #Generates a random 2D grid map for path planning
    #Provide the w, h, density and seed for reproductability
    #Output is the grid, the start and the goal positions

    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    grid = np.zeros((height, width), dtype=int)

    # Randomly assign obstacles based on obstacle density
    num_obstacles = int(obstacle_density * width * height)
    obstacle_indices = random.sample(range(width * height), num_obstacles)
    for idx in obstacle_indices:
        x, y = idx % width, idx // width
        grid[y][x] = 1

    # Pick random start and goal positions in free cells
    free_cells = list(zip(*np.where(grid == 0)))
    start = random.choice(free_cells)
    goal = random.choice(free_cells)
    while goal == start:
        goal = random.choice(free_cells)

    return grid, start, goal

grid, start, goal = generate_random_grid(20, 20, 0.25, None)

def neighbors(pos, nrows, ncols):
    r, c = pos
    for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < nrows and 0 <= nc < ncols:
            yield (nr, nc)


def min_constraint_removal(grid, start, goal):
    nrows, ncols = grid.shape
    pq = [(0, start)]  # (obstacles_removed, position)
    visited = {start: 0}
    parent = {start: None}

    while pq:
        removed, pos = heapq.heappop(pq)
        if pos == goal:
            # reconstruct path
            path = []
            while pos:
                path.append(pos)
                pos = parent[pos]
            return path[::-1], removed

        for n in neighbors(pos, nrows, ncols):
            cost = removed + grid[n]  # +1 if obstacle
            if n not in visited or cost < visited[n]:
                visited[n] = cost
                parent[n] = pos
                heapq.heappush(pq, (cost, n))
    return None, float('inf')

path, removed = min_constraint_removal(grid, start, goal)
print(f"Minimum obstacles to remove: {removed}")

# Visualization
plt.imshow(grid, cmap='gray_r')
if path:
    r, c = zip(*path)
    plt.plot(c, r, color='blue')
plt.scatter(start[1], start[0], color='green', label='Start')
plt.scatter(goal[1], goal[0], color='red', label='Goal')
plt.legend()
plt.show()
