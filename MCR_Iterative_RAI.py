import robotic as ry
import numpy as np
import heapq
import itertools
import random
import time
from math import sqrt, atan2, cos, sin


# --- GRAPH NODE CLASS ---
class GraphNode:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.neighbors = []  # List of (neighbor_node, edge_cost, edge_disps)

    def __lt__(self, other):
        # Tie-breaker for priority queue
        return id(self) < id(other)


# --- PLANNER CLASS ---
class IterativeMCDPlannerRAI:
    def __init__(self, g_filename, step_size=0.1, penalty=500.0, goal_bias=0.05, connection_radius=0.3):
        # 1. Initialize RAI Config
        self.C = ry.Config()
        print(f"Loading config from: {g_filename}")
        self.C.addFile(g_filename)

        # 2. Setup Robot and Goal
        self.robot_name = "ego"
        self.goal_frame_name = "target"

        self.step_size = step_size
        self.penalty = penalty
        self.goal_bias = goal_bias
        self.w_L = 0.001  # Small weight for path length
        self.connection_radius = connection_radius

        # Get Start and Goal Positions (Assuming 2D navigation on XY plane for simplicity, keeping Z constant)
        start_frame = self.C.getFrame(self.robot_name)
        if not start_frame:
            raise ValueError(f"Robot frame '{self.robot_name}' not found.")
        self.start_pos_3d = start_frame.getPosition()
        self.start_pos_2d = self.start_pos_3d[:2]
        self.robot_z = self.start_pos_3d[2]  # Maintain constant Z height

        goal_frame = self.C.getFrame(self.goal_frame_name)
        if not goal_frame:
            raise ValueError(f"Goal frame '{self.goal_frame_name}' not found.")
        self.goal_pos_3d = goal_frame.getPosition()
        self.goal_pos_2d = self.goal_pos_3d[:2]

        # 3. Identify Obstacles
        self.soft_obstacles = []
        self.hard_obstacles = []

        for frame in self.C.getFrames():
            name = frame.name
            # Convention: 'obj' prefix is movable, 'table' or 'wall' is hard
            if name.startswith("obj"):
                self.soft_obstacles.append(name)
            elif name.startswith("table") or name.startswith("wall"):
                self.hard_obstacles.append(name)

        # 4. Initialize Graph with Start and Goal Nodes
        self.start_node = GraphNode(self.start_pos_2d)
        self.goal_node = GraphNode(self.goal_pos_2d)
        self.nodes = [self.start_node, self.goal_node]

        # Dictionary to track discovered constraints
        self.obstacles_D = {obs: [0] for obs in self.soft_obstacles}
        self.tie_breaker = itertools.count()

        # Define bounds for random sampling
        # You might need to adjust these based on your specific .g file
        self.x_min, self.x_max = -1.0, 1.0
        self.y_min, self.y_max = -1.0, 1.0

    def get_random_point(self):
        if random.random() < self.goal_bias:
            return self.goal_pos_2d

        return np.array([
            random.uniform(self.x_min, self.x_max),
            random.uniform(self.y_min, self.y_max)
        ])

    def dist(self, p1, p2):
        return np.linalg.norm(p1 - p2)

    def set_robot_pos(self, pos_2d):
        """Helper to move the robot frame in the simulation for collision checking."""
        self.C.getFrame(self.robot_name).setPosition([pos_2d[0], pos_2d[1], self.robot_z])
        self.C.computeCollisions()

    def check_collision(self, pos_2d):
        """
        Checks for HARD collisions at a specific point.
        Returns True if colliding with a hard obstacle.
        """
        self.set_robot_pos(pos_2d)
        collisions = self.C.getCollisions()
        for col in collisions:
            # col is (frameA, frameB, distance, ...)
            # Check if collision involves robot and a hard obstacle
            # Note: RAI collisions are usually reported if distance < 0
            if col[2] < 0:
                f1, f2 = col[0], col[1]
                if (f1 == self.robot_name and f2 in self.hard_obstacles) or \
                        (f2 == self.robot_name and f1 in self.hard_obstacles):
                    return True
        return False

    def get_edge_data(self, from_pos, to_pos):
        """
        Checks an edge for validity and calculates cost.
        Returns (total_cost, dict_of_soft_constraints)
        """
        path_length = self.dist(from_pos, to_pos)
        steps = int(path_length / (self.step_size * 0.5)) + 1  # finer resolution for checking

        edge_disp_cost = 0
        edge_disps = {}
        found_soft_obs = set()

        # Interpolate along the line
        for i in range(steps + 1):
            t = i / steps
            curr_pos = from_pos + t * (to_pos - from_pos)

            self.set_robot_pos(curr_pos)
            collisions = self.C.getCollisions()

            for col in collisions:
                if col[2] < 0:  # Actual collision
                    f1, f2 = col[0], col[1]
                    other = f2 if f1 == self.robot_name else (f1 if f2 == self.robot_name else None)

                    if other:
                        if other in self.hard_obstacles:
                            return float('inf'), {}  # Hard collision, invalid edge
                        elif other in self.soft_obstacles:
                            found_soft_obs.add(other)

        # Calculate penalties for unique soft obstacles encountered
        for obs in found_soft_obs:
            edge_disp_cost += self.penalty
            edge_disps[obs] = 1
            if 1 not in self.obstacles_D[obs]:
                self.obstacles_D[obs].append(1)

        total_cost = (self.w_L * path_length) + edge_disp_cost
        return total_cost, edge_disps

    def grow_graph(self, n_samples):
        print(f"Growing graph by {n_samples} samples...")
        for _ in range(n_samples):
            q_rand = self.get_random_point()

            # Find nearest node
            q_near = min(self.nodes, key=lambda n: self.dist(n.pos, q_rand))

            # Steer (Greedy extension not fully implemented here to keep simple, just single step or until blocked)
            # Standard Step:
            direction = q_rand - q_near.pos
            dist = np.linalg.norm(direction)
            if dist == 0: continue

            direction = direction / dist
            q_new_pos = q_near.pos + direction * min(dist, self.step_size)

            # Check bounds (optional if implicit by obstacles)
            if not (self.x_min <= q_new_pos[0] <= self.x_max and self.y_min <= q_new_pos[1] <= self.y_max):
                continue

            # Hard collision check for the node itself
            if self.check_collision(q_new_pos):
                continue

            # Add Node
            new_node = GraphNode(q_new_pos)
            self.nodes.append(new_node)

            # Connect to neighbors (PRM style)
            neighbors = [n for n in self.nodes if self.dist(n.pos, new_node.pos) <= self.connection_radius]

            for neighbor in neighbors:
                cost, disps = self.get_edge_data(neighbor.pos, new_node.pos)
                if cost != float('inf'):
                    neighbor.neighbors.append((new_node, cost, disps))
                    new_node.neighbors.append((neighbor, cost, disps))

    def run_astar(self):
        start_h = self.dist(self.start_node.pos, self.goal_pos_2d) * self.w_L
        pq = [(start_h, 0, next(self.tie_breaker), self.start_node, None, {})]
        visited_costs = {}
        came_from = {}

        while pq:
            f, g, _, current, parent, disps = heapq.heappop(pq)

            if current in visited_costs and visited_costs[current] <= g: continue
            visited_costs[current] = g
            came_from[current] = (parent, disps)

            if current == self.goal_node:
                return self.reconstruct_path(came_from, current, g)

            for neighbor, edge_cost, edge_disps in current.neighbors:
                new_g = g + edge_cost
                if neighbor not in visited_costs or new_g < visited_costs[neighbor]:
                    h = self.dist(neighbor.pos, self.goal_pos_2d) * self.w_L
                    heapq.heappush(pq, (new_g + h, new_g, next(self.tie_breaker), neighbor, current, edge_disps))

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
            "removed_constraints": list(set(all_disps.keys())),
            "soft_violations": len(set(all_disps.keys()))
        }

    def plan(self, max_iters=2000, check_interval=200, stability_threshold=2):
        print(f"Starting Iterative Planning (Max: {max_iters}, Interval: {check_interval})...")

        best_path_result = None
        best_cost = float('inf')
        unchanged_count = 0

        total_batches = max_iters // check_interval

        for batch in range(1, total_batches + 1):
            current_iter_count = batch * check_interval

            # 1. Grow Graph
            self.grow_graph(check_interval)

            # 2. Run A*
            result = self.run_astar()

            if result:
                cost = result['total_cost']
                removals = result['soft_violations']
                print(f"  [Iter {current_iter_count}] Path found. Cost: {cost:.2f} (Removals: {removals})")

                if removals == 0:
                    print("  -> 0-removal path found! Terminating early.")
                    return result

                if cost < best_cost:
                    best_cost = cost
                    best_path_result = result
                    unchanged_count = 0
                elif cost == best_cost:
                    unchanged_count += 1
                    print(f"  -> Solution stable for {unchanged_count}/{stability_threshold} checks.")
                    if unchanged_count >= stability_threshold:
                        print("  -> Solution stabilized. Terminating early.")
                        return best_path_result
            else:
                print(f"  [Iter {current_iter_count}] No path found yet.")

        print("Max iterations reached.")
        return best_path_result


# --- MAIN EXECUTION ---
if __name__ == "__main__":
    # Assumes a .g file exists. Replace with your actual file path.
    G_FILE = "scene.g"

    try:
        planner = IterativeMCDPlannerRAI(G_FILE,
                                         step_size=0.2,
                                         penalty=100.0,
                                         goal_bias=0.1,
                                         connection_radius=0.5)

        result = planner.plan(max_iters=1000, check_interval=100, stability_threshold=2)

        if result:
            print("=" * 40)
            print(f"Path Found!")
            print(f"Removals Needed: {result['soft_violations']}")
            print(f"Objects: {result['removed_constraints']}")
            print("=" * 40)

            # Visualization
            for i, p in enumerate(result["path"]):
                planner.C.addFrame(f"path_{i}") \
                    .setShape(ry.ST.sphere, [0.05]) \
                    .setPosition([p[0], p[1], planner.robot_z]) \
                    .setColor([0, 1, 0])

            # Highlight removed objects
            for obj in result['removed_constraints']:
                f = planner.C.getFrame(obj)
                if f: f.setColor([1, 0, 0])

            planner.C.view(True, "Final Path")

            # Simple Animation
            for p in result["path"]:
                planner.set_robot_pos(p)
                time.sleep(0.05)

        else:
            print("No path found.")

    except Exception as e:
        print(f"Error: {e}")