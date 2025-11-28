import matplotlib

matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
import heapq
import itertools
import random
from math import sqrt, atan2, cos, sin

#The same as MCR-Sampling
#But the smapling is improved by a greedy technique
#Instead of taking one small step towards a random point and stopping
#The new logic keeps going towards the random point until it
# either reaches it or hits a hard obstacle
#This allows us to shoot long branches into open spaces
#In the previous sampling algorithm, when the point was far away, it was ignored essentially
#Added shortcut techniques and changed the planner to use floats as well

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


class Node:
    def __init__(self, pos, parent=None):
        self.pos = pos
        self.parent = parent
        self.children = []

    def __lt__(self, other):
        return id(self) < id(other)


class MCDSamplingPlanner:
    def __init__(self, grid, start, goal, step_size=1, max_iters=5000, penalty=500, goal_bias=0.1):
        self.grid = grid
        self.start_node = Node(start)
        self.goal_pos = goal
        self.step_size = step_size
        self.max_iters = max_iters
        self.penalty = penalty
        self.goal_bias = goal_bias
        self.w_L = 0.0001

        self.nodes = [self.start_node]
        self.soft_obstacles = set(zip(*np.where(grid == 2)))
        self.obstacles_D = {obs: [0] for obs in self.soft_obstacles}
        self.tie_breaker = itertools.count()

    def get_random_point(self):
        if random.random() < self.goal_bias:
            return self.goal_pos
        return (random.randint(0, self.grid.shape[0] - 1),
                random.randint(0, self.grid.shape[1] - 1))

    def nearest(self, point):
        return min(self.nodes, key=lambda n: (n.pos[0] - point[0]) ** 2 + (n.pos[1] - point[1]) ** 2)

    def dist(self, p1, p2):
        return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def check_edge_hard_collision(self, from_pos, to_pos):
        line = bresenham_line(*from_pos, *to_pos)
        for (x, y) in line:
            if x < 0 or y < 0 or x >= self.grid.shape[0] or y >= self.grid.shape[1]:
                return True
            if self.grid[x, y] == 1:  # Hard obstacle
                return True
        return False

    def _phase2_sample_displacement(self, from_pos, to_pos):
        line = bresenham_line(*from_pos, *to_pos)
        path_set = set(line)
        colliding_soft_obs = path_set.intersection(self.soft_obstacles)
        for obs in colliding_soft_obs:
            if 1 not in self.obstacles_D[obs]:
                self.obstacles_D[obs].append(1)

    def plan(self):
        print(f"Building roadmap ({self.max_iters} iterations)...")

        for i in range(self.max_iters):
            q_rand = self.get_random_point()
            q_near = self.nearest(q_rand)
            curr_node = q_near

            # --- FIX: Track position as float to allow diagonal movement ---
            curr_x, curr_y = float(curr_node.pos[0]), float(curr_node.pos[1])

            # Greedy extend loop
            while self.dist(curr_node.pos, q_rand) > 0.5:
                # Calculate direction
                theta = atan2(q_rand[1] - curr_node.pos[1], q_rand[0] - curr_node.pos[0])

                # Update float position
                curr_x += self.step_size * cos(theta)
                curr_y += self.step_size * sin(theta)

                # Convert to int for grid checking (Round instead of floor)
                q_new_pos = (int(round(curr_x)), int(round(curr_y)))

                if q_new_pos == curr_node.pos:
                    continue  # Moved less than a pixel, keep accumulating float

                # Check Bounds
                if not (0 <= q_new_pos[0] < self.grid.shape[0] and 0 <= q_new_pos[1] < self.grid.shape[1]):
                    break

                # 1. Check Hard Collisions
                if self.check_edge_hard_collision(curr_node.pos, q_new_pos):
                    break

                    # 2. Add Node
                new_node = Node(q_new_pos, parent=curr_node)
                curr_node.children.append(new_node)
                self.nodes.append(new_node)

                # 3. Sample Displacements
                self._phase2_sample_displacement(curr_node.pos, q_new_pos)

                curr_node = new_node

        print(f"Roadmap built with {len(self.nodes)} nodes. Running MCD solver...")

        path, displacements, cost = self.run_discrete_mcd_solver()

        # --- FIX: Run Shortcut Optimizer ---
        if path:
            print(f"Original path cost: {cost:.2f}. optimizing...")
            path, displacements, cost = self.shortcut_path(path)

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

    def find_cheapest_edge_cost(self, from_pos, to_pos):
        line = bresenham_line(*from_pos, *to_pos)
        path_length = self.dist(from_pos, to_pos)
        edge_disp_cost = 0
        edge_disps = {}

        # Hard collision check
        for (x, y) in line:
            if self.grid[x, y] == 1:
                return float('inf'), {}

        # Soft collision check
        path_set = set(line)
        colliding_soft_obs = path_set.intersection(self.soft_obstacles)

        for obs in colliding_soft_obs:
            # We assume if we are shortcutting, we are allowed to remove anything
            # that was DISCOVERED during the RRT phase.
            available_disps = self.obstacles_D[obs]
            if 1 in available_disps:
                edge_disp_cost += self.penalty
                edge_disps[obs] = 1
            else:
                return float('inf'), {}

        total_edge_cost = (self.w_L * path_length) + edge_disp_cost
        return total_edge_cost, edge_disps

    def run_discrete_mcd_solver(self):
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
                print(f"Solver found goal node with cost {J_cost:.4f}.")
                return self.reconstruct_mcd_path(came_from, node, J_cost)

            for child in node.children:
                edge_J_cost, edge_disps_to_child = self.find_cheapest_edge_cost(node.pos, child.pos)
                if edge_J_cost == float('inf'): continue

                new_J_cost = J_cost + edge_J_cost
                if new_J_cost >= visited_costs.get(child, float('inf')): continue

                h = self.dist(child.pos, self.goal_pos) * self.w_L
                heapq.heappush(pq,
                               (new_J_cost + h, new_J_cost, next(self.tie_breaker), child, node, edge_disps_to_child))

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

    # --- NEW FUNCTION: SHORTCUTTING ---
    def shortcut_path(self, path, iterations=150):
        """Attempts to skip waypoints to reduce cost (removals + length)."""
        if len(path) < 3:
            return path, {}, 0  # Can't shortcut

        # Re-calculate initial cost/disps
        current_path = list(path)

        for _ in range(iterations):
            if len(current_path) < 3: break

            # Pick two random indices
            idx_a = random.randint(0, len(current_path) - 2)
            idx_b = random.randint(idx_a + 1, len(current_path) - 1)

            pos_a = current_path[idx_a]
            pos_b = current_path[idx_b]

            # Check direct connection cost
            shortcut_cost, shortcut_disps = self.find_cheapest_edge_cost(pos_a, pos_b)

            if shortcut_cost == float('inf'):
                continue

            # Calculate cost of the EXISTING segment between A and B
            existing_cost = 0
            for i in range(idx_a, idx_b):
                c, _ = self.find_cheapest_edge_cost(current_path[i], current_path[i + 1])
                existing_cost += c

            # If shortcut is cheaper, replace the segment
            if shortcut_cost < existing_cost:
                # Replace the middle part with nothing (direct connection)
                current_path = current_path[:idx_a + 1] + current_path[idx_b:]

        # Recalculate final stats for the shortcut path
        final_cost = 0
        final_disps = {}
        for i in range(len(current_path) - 1):
            c, d = self.find_cheapest_edge_cost(current_path[i], current_path[i + 1])
            final_cost += c
            final_disps.update(d)

        return current_path, final_disps, final_cost


if __name__ == "__main__":
    # Use the same seed to verify the fix
    grid = generate_random_grid(size=20, obstacle_density=0.25, soft_ratio=0.4, seed=115)
    start = (0, 0)
    goal = (18, 19)

    planner = MCDSamplingPlanner(grid, start, goal,
                                 step_size=1,
                                 max_iters=5000,
                                 penalty=50000,
                                 goal_bias=0.10)

    result = planner.plan()

    plt.figure(figsize=(8, 8))
    plt.imshow(grid.T, origin="lower", cmap="gray_r")
    plt.scatter(*start, c='green', s=100, label="Start")
    plt.scatter(*goal, c='blue', s=100, label="Goal")

    for node in planner.nodes:
        if node.parent:
         p1, p2 = node.parent.pos, node.pos
         plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'g-', alpha=0.1)

    if result:
        path = result["path"]
        xs, ys = zip(*path)
        plt.plot(xs, ys, 'r-', linewidth=2, label="Path")
        if result["removed_constraints"]:
            r_xs, r_T = zip(*result["removed_constraints"])
            plt.scatter(r_xs, r_T, c='orange', s=60, marker='x', label="Removed")
        print("\n Path found!")
        print(f"  Total Cost: {result['total_cost']:.2f}")
        print(f"  Soft Constraints Removed: {result['soft_violations']}")
    else:
        print("\n No path found")

    plt.legend()
    plt.show()