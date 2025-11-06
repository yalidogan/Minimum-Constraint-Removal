import matplotlib

matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
import heapq
import itertools
import random
from math import sqrt

#This is a proper test document for the MCR implementation
#Here I have implemented a sampling based MCR implementation based on some articles
#The algorithm using the minheap of the implementation before was a greedy approach
#It could get stuck on local minima and was not that much of an improvement
#Here the algorithm works as such:

#First build a roadmap to the goal:
#Uses RRT
#pick a random point off the grid, or pick goal if its goal biased
#find node closest in the tree to q_random
#If q_random is the goal we try to connect all the way
#o/w we steer one stepsize from q_n to q_rand to get a new q_rand
#If it hits any hard collisions we discard this new point and start the loop again
#Completely ignore soft obstacles for now!

#This part happens in step 1 right before adding the new edge to the tree
#Look at the valid edge we are about to create from q_n to q_new
#Scan the edge and find the soft constraints
#For every soft constr we update the master list
#Add a remove option to its list of available disp
#0 mean keep 1 mean remove

#THEN: We find the optimal path with an A* solver:
#We run an A* search on the RRT TREE not the whole graph
#a*'s goal is to find the path from the start node to any goal that minimizes:
#total length + total penalty
#It removes any of the soft constr on top of the path

#The A* search finds the path through the RRT tree
#that has the cheapest combination of length and removal penalties

#The come from dictionary allows reconstruction and see which constr are removed



# ==============================
# Generate the Test Environment
# ==============================
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


# ========================================
# Bresenham Line
# ========================================
def bresenham_line(x1, y1, x2, y2):
    #Generate grid cells on a line for collision check
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    points = []
    #Below coord code is very error prone
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


# ==============================
# NODE class -> very simple
# ==============================
class Node:
    def __init__(self, pos, parent=None):
        self.pos = pos
        self.parent = parent
        self.children = []

    def __lt__(self, other):
        #Allows for the comparison of nodes
        return id(self) < id(other)


# ==============================
# MCD SAMPLING PLANNER
# ==============================
class MCDSamplingPlanner:
    def __init__(self, grid, start, goal, step_size=3, max_iters=5000, penalty=5, goal_bias=0.1):
        self.grid = grid
        self.start_node = Node(start)
        self.goal_pos = goal
        self.step_size = step_size
        self.max_iters = max_iters
        self.penalty = penalty  # cost to remove a constr
        self.goal_bias = goal_bias
        self.w_L = 1.0  # Weight for path length

        self.nodes = [self.start_node]  # Set of all nodes in the roadmap

        # Identify all soft obstacles
        self.soft_obstacles = set(zip(*np.where(grid == 2)))

        # self.obstacles_D: { (x,y): [list_of_displacements] }
        # 0 = keep obstacle (cost 0), 1 = remove obstacle (cost self.penalty)
        self.obstacles_D = {obs: [0] for obs in self.soft_obstacles}

        self.tie_breaker = itertools.count()

    def get_random_point(self):
        if random.random() < self.goal_bias:
            return self.goal_pos
        return (random.randint(0, self.grid.shape[0] - 1),
                random.randint(0, self.grid.shape[1] - 1))

    def nearest(self, point):
        return min(self.nodes, key=lambda n: self.dist(n.pos, point))

    def steer(self, from_pos, to_point):
        x1, y1 = from_pos
        x2, y2 = to_point
        dx, dy = x2 - x1, y2 - y1
        dist = self.dist(from_pos, to_point)
        if dist == 0:
            return from_pos
        scale = min(self.step_size / dist, 1)
        new_pos = (int(x1 + dx * scale), int(y1 + dy * scale))
        return new_pos

    def dist(self, p1, p2):
        return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def check_edge_hard_collision(self, from_pos, to_pos):
        #To only check for hard constr collisions
        line = bresenham_line(*from_pos, *to_pos)
        for (x, y) in line:
            if x < 0 or y < 0 or x >= self.grid.shape[0] or y >= self.grid.shape[1]:
                return True  # outside grid
            if self.grid[x, y] == 1:  # hard constraint
                return True
        return False

    def plan(self):

        # build the roadmap and the sample displacement
        print(f"Building roadmap ({self.max_iters} iterations)...")
        for i in range(self.max_iters):
            q_rand = self.get_random_point()
            q_n = self.nearest(q_rand)

            # If goal-biased, try to connect all the way to the goal
            if q_rand == self.goal_pos:
                q_new_pos = self.goal_pos
            else:
                q_new_pos = self.steer(q_n.pos, q_rand)

            if q_new_pos == q_n.pos:
                continue

            if not self.check_edge_hard_collision(q_n.pos, q_new_pos):
                #Sample displacement
                self._phase2_sample_displacement(q_n.pos, q_new_pos)

                #Add the new node to a tree
                new_node = Node(q_new_pos, parent=q_n)
                q_n.children.append(new_node)
                self.nodes.append(new_node)

        print("Roadmap built. Running MCD solver...")

        #Run the discrete MCR solver
        path, displacements, cost = self.run_discrete_mcd_solver()

        if path:
            path_length = 0
            for i in range(len(path) - 1):
                path_length += self.dist(path[i], path[i + 1])

            return {
                "path": path,
                "total_cost": cost,
                "path_length": path_length,
                "soft_violations": len(displacements),
                "removed_constraints": set(displacements.keys())
            }
        return None

    def _phase2_sample_displacement(self, from_pos, to_pos):
        #Finds a soft obst on the edge and sample a remove
        line = bresenham_line(*from_pos, *to_pos)
        colliding_soft_obs = {obs for obs in line if obs in self.soft_obstacles}

        for obs in colliding_soft_obs:
            if 1 not in self.obstacles_D[obs]:
                self.obstacles_D[obs].append(1)  # 1 = "remove"

    def find_cheapest_edge_cost(self, from_pos, to_pos):
        #Calculates the min cost to traverse an edge with curr sample displacement
        line = bresenham_line(*from_pos, *to_pos)
        path_length = self.dist(from_pos, to_pos)
        edge_disp_cost = 0
        edge_disps = {}

        for (x, y) in line:
            if self.grid[x, y] == 1:
                return float('inf'), {}

        colliding_soft_obs = {obs for obs in line if obs in self.soft_obstacles}

        for obs in colliding_soft_obs:
            available_disps = self.obstacles_D[obs]

            if 1 in available_disps:
                edge_disp_cost += self.penalty
                edge_disps[obs] = 1
            else:
                return float('inf'), {}

        total_edge_cost = (self.w_L * path_length) + edge_disp_cost
        return total_edge_cost, edge_disps

    def run_discrete_mcd_solver(self):
        #Runs an A* search on the RRT roadmap to find the cheapest path from start to goal

        start_h = self.dist(self.start_node.pos, self.goal_pos) * self.w_L
        pq = [(start_h, 0, next(self.tie_breaker), self.start_node, None, {})]

        visited_costs = {}
        came_from = {}

        while pq:
            f_cost, J_cost, _, node, parent, edge_disps = heapq.heappop(pq)

            if node in visited_costs and visited_costs[node] <= J_cost:
                continue

            visited_costs[node] = J_cost
            came_from[node] = (parent, edge_disps)

            if node.pos == self.goal_pos:
                print(f"Solver found goal node with cost {J_cost}.")
                return self.reconstruct_mcd_path(came_from, node, J_cost)

            for child in node.children:
                # I don't check visited_costs here.
                # Let heap do that

                edge_J_cost, edge_disps_to_child = self.find_cheapest_edge_cost(node.pos, child.pos)

                if edge_J_cost == float('inf'):
                    continue

                new_J_cost = J_cost + edge_J_cost

                # must check if this new path to the child is better
                # than any other path already found to it.
                if new_J_cost >= visited_costs.get(child, float('inf')):
                    continue  # Not a better path dont add

                h = self.dist(child.pos, self.goal_pos) * self.w_L
                new_f_cost = new_J_cost + h

                heapq.heappush(pq, (new_f_cost, new_J_cost, next(self.tie_breaker), child, node, edge_disps_to_child))

        print("Solver could not find a path to any goal node.")
        return None, {}, float('inf')

    def reconstruct_mcd_path(self, came_from, goal_node, cost):
        path = []
        all_disps = {}
        curr_node = goal_node

        while curr_node is not None:
            path.append(curr_node.pos)
            parent_node, edge_disps = came_from[curr_node]
            all_disps.update(edge_disps)
            curr_node = parent_node

        path.reverse()
        final_disps = {obs: disp for obs, disp in all_disps.items() if disp == 1}
        return path, final_disps, cost


# ==============================
# RUN TEST
# ==============================

if __name__ == "__main__":
    grid = generate_random_grid(size=30, obstacle_density=0.25, soft_ratio=0.4, seed=10)
    start = (4, 5)
    goal = (25, 25)

    planner = MCDSamplingPlanner(grid, start, goal,
                                 step_size=3,
                                 max_iters=3000,
                                 penalty=10,
                                 goal_bias=0.15)

    result = planner.plan()

    plt.figure(figsize=(8, 8))
    plt.imshow(grid.T, origin="lower", cmap="gray_r")

    plt.scatter(*start, c='green', s=80, label="Start")
    plt.scatter(*goal, c='blue', s=80, label="Goal")

    if result:
        path = result["path"]
        xs, ys = zip(*path)
        plt.plot(xs, ys, 'r-', linewidth=2, label="Path")

        if result["removed_constraints"]:
            r_xs, r_T = zip(*result["removed_constraints"])
            plt.scatter(r_xs, r_T, c='orange', s=50, marker='x', label="Removed Constraints")
        else:
            plt.scatter([], [], c='orange', s=50, marker='x', label="Removed Constraints")

        print("\n Path found!")
        print(f"  Total Cost: {result['total_cost']:.2f}")
        print(f"  Path Length: {result['path_length']:.2f}")
        print(f"  Soft Constraints Removed: {result['soft_violations']}")
    else:
        print("\n No path found")

    plt.legend()
    plt.show()