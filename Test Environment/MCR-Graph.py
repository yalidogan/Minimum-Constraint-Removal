import matplotlib

matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
import heapq
import itertools
import random
from math import sqrt, atan2, cos, sin


#Converted the RRT tree to a PRM like graph
#There are now alternative paths for the A* to take
#There are cycles and more connections for the nodes
#Added a greedy approach of explore until hard obstacle to explore open spaces
#Corrected the squeezing through corners

def generate_random_grid(size=30, obstacle_density=0.25, soft_ratio=0.4, seed=None):
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


class GraphNode:
    def __init__(self, pos):
        self.pos = pos
        self.neighbors = []  # List of (neighbor_node, edge_cost, edge_disps)

    def __lt__(self, other):
        return id(self) < id(other)


class MCDSamplingPlanner:
    def __init__(self, grid, start, goal, step_size=1, max_iters=2000, penalty=5000, goal_bias=0.05,
                 connection_radius=3):
        self.grid = grid
        self.start_node = GraphNode(start)
        self.goal_pos = goal

        #for RRT to connect directly to the goal pos
        self.goal_node = GraphNode(goal)

        self.step_size = step_size
        self.max_iters = max_iters
        self.penalty = penalty
        self.goal_bias = goal_bias
        self.w_L = 0.001
        self.connection_radius = connection_radius

        # Initialize graph with start and goal
        self.nodes = [self.start_node, self.goal_node]

        self.soft_obstacles = set(zip(*np.where(grid == 2)))
        self.obstacles_D = {obs: [0] for obs in self.soft_obstacles}
        self.tie_breaker = itertools.count()

    def get_random_point(self):
        if random.random() < self.goal_bias:
            return self.goal_pos
        return (random.randint(0, self.grid.shape[0] - 1),
                random.randint(0, self.grid.shape[1] - 1))

    def dist(self, p1, p2):
        return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def check_edge_hard_collision(self, from_pos, to_pos):
        line = bresenham_line(*from_pos, *to_pos)

        #are any cells on the line hard obstacles
        for (x, y) in line:
            if x < 0 or y < 0 or x >= self.grid.shape[0] or y >= self.grid.shape[1]:
                return True
            if self.grid[x, y] == 1:
                return True

        #koşelerin arasından sıkışmasın diye
        #robotun hacmi var gibi adeta hesaplamalar yap

        for i in range(len(line) - 1):
            p1 = line[i]
            p2 = line[i + 1]

            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]

            # If moving diagonally (both x and y change)
            if abs(dx) == 1 and abs(dy) == 1:
                #check the two cardinal neighbors in transition
                #if we move (0,0) -> (1,1)
                #we check (0,1) and (1,0)
                n1 = (p1[0] + dx, p1[1])  # Neighbor 1 horizontal
                n2 = (p1[0], p1[1] + dy)  # Neighbor 2 vertical

                #if either cardinal neighbor is a hard obstacle the diagonal is illegal
                #as if the robot has volume
                if self.grid[n1] == 1 or self.grid[n2] == 1:
                    return True

        return False

    def get_edge_data(self, from_pos, to_pos):

        #run strict hard collision check
        if self.check_edge_hard_collision(from_pos, to_pos):
            return float('inf'), {}

        #If safe calc soft constaint costs
        line = bresenham_line(*from_pos, *to_pos)
        path_length = self.dist(from_pos, to_pos)
        edge_disp_cost = 0
        edge_disps = {}

        path_set = set(line)
        colliding_soft_obs = path_set.intersection(self.soft_obstacles)

        for obs in colliding_soft_obs:
            if 1 not in self.obstacles_D[obs]:
                self.obstacles_D[obs].append(1)

            edge_disp_cost += self.penalty
            edge_disps[obs] = 1

        total_cost = (self.w_L * path_length) + edge_disp_cost
        return total_cost, edge_disps

    def plan(self):
        print(f"Building GRAPH roadmap ({self.max_iters} iterations)...")

        for i in range(self.max_iters):
            q_rand = self.get_random_point()

            #find nearest node
            q_near = min(self.nodes, key=lambda n: self.dist(n.pos, q_rand))

            #steer
            theta = atan2(q_rand[1] - q_near.pos[1], q_rand[0] - q_near.pos[0])
            #float for precise movement measures
            new_x_float = q_near.pos[0] + self.step_size * cos(theta)
            new_y_float = q_near.pos[1] + self.step_size * sin(theta)
            q_new_pos = (int(round(new_x_float)), int(round(new_y_float)))

            if q_new_pos == q_near.pos: continue
            if not (0 <= q_new_pos[0] < self.grid.shape[0] and 0 <= q_new_pos[1] < self.grid.shape[1]): continue

            #check hard collision
            if self.check_edge_hard_collision(q_near.pos, q_new_pos):
                continue

            #create new node
            new_node = GraphNode(q_new_pos)
            self.nodes.append(new_node)

            #connect to neighbors
            #connect to goal node if close
            neighbors = [n for n in self.nodes if self.dist(n.pos, new_node.pos) <= self.connection_radius]

            for neighbor in neighbors:
                cost, disps = self.get_edge_data(neighbor.pos, new_node.pos)
                if cost != float('inf'):
                    neighbor.neighbors.append((new_node, cost, disps))
                    new_node.neighbors.append((neighbor, cost, disps))

        print(f"Graph built with {len(self.nodes)} nodes. Running A*...")
        return self.run_astar()

    def run_astar(self):
        # We now search for self.goal_node
        start_h = self.dist(self.start_node.pos, self.goal_pos) * self.w_L
        pq = [(start_h, 0, next(self.tie_breaker), self.start_node, None, {})]

        visited_costs = {}
        came_from = {}

        while pq:
            f, g, _, current, parent, disps = heapq.heappop(pq)

            if current in visited_costs and visited_costs[current] <= g:
                continue
            visited_costs[current] = g
            came_from[current] = (parent, disps)

            # Exact match check
            #goal_node is in the graph
            if current == self.goal_node:
                print(f"Solver found exact goal with cost {g:.4f}.")
                return self.reconstruct_path(came_from, current, g)

            for neighbor, edge_cost, edge_disps in current.neighbors:
                new_g = g + edge_cost
                if neighbor not in visited_costs or new_g < visited_costs[neighbor]:
                    h = self.dist(neighbor.pos, self.goal_pos) * self.w_L
                    heapq.heappush(pq, (new_g + h, new_g, next(self.tie_breaker), neighbor, current, edge_disps))

        print("A* failed to reach the goal node.")
        return None

    def reconstruct_path(self, came_from, current, cost):
        path = []
        all_disps = {}
        while current:
            path.append(current.pos)
            parent, disps = came_from[current]
            all_disps.update(disps)
            current = parent
        path.reverse()

        return {
            "path": path,
            "total_cost": cost,
            "removed_constraints": set(all_disps.keys()),
            "soft_violations": len(all_disps)
        }


if __name__ == "__main__":

    grid = generate_random_grid(size=20, obstacle_density=0.25, soft_ratio=0.4, seed=15)
    start = (0, 0)
    goal = (18, 19)

    planner = MCDSamplingPlanner(grid, start, goal,
                                 step_size=1,
                                 max_iters=5000,
                                 connection_radius=4,
                                 penalty=5000,
                                 goal_bias=0.05)

    result = planner.plan()

    plt.figure(figsize=(8, 8))
    plt.imshow(grid.T, origin="lower", cmap="gray_r")
    plt.scatter(*start, c='green', s=100, label="Start")
    plt.scatter(*goal, c='blue', s=100, label="Goal")

    if result:
        path = result["path"]
        xs, ys = zip(*path)
        plt.plot(xs, ys, 'r-', linewidth=2, label="Path")
        if result["removed_constraints"]:
            r_xs, r_T = zip(*result["removed_constraints"])
            plt.scatter(r_xs, r_T, c='orange', s=60, marker='x', label="Removed")
        print(f"\nPath found! Removals: {result['soft_violations']}")
    else:
        print("\nNo path found")

    plt.legend()
    plt.show()