import robotic as ry
import numpy as np
import heapq
import itertools
import random
import time

class GraphNode:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.neighbors = []  # List of (neighbor, cost, set_of_constraints)

    def __lt__(self, other):
        return id(self) < id(other)

class RAIMCRPlanner:
    def __init__(self, g_filename, penalty=500.0):
        # 1. Initialize RAI Config
        self.C = ry.Config()
        print(f"Loading config from: {g_filename}")
        self.C.addFile(g_filename)
        self.C.computeCollisions()
        
        #setup the robot and the goal 
        self.robot_name = "ego"
        self.penalty = penalty
        
        #Use getPosition here 
        #getJointState is not compatible 
        #Set the goal and start frames below 

        start_frame = self.C.getFrame(self.robot_name)
        if not start_frame:
             raise ValueError(f"Robot frame '{self.robot_name}' not found. Check your .g file.")
             
        start_3d = start_frame.getPosition()
        
        #Store the z height 
        #maintain it while moving x and y 

        self.robot_z = start_3d[2]
        self.start_pos = np.array([start_3d[0], start_3d[1]])
        
        #Get the goal pos
        #Can change the goal pos to anything of the liking 
        #Change the name to the correct objective in the g file or as the one you set 
        #Center of goal1

        goal_frame = self.C.getFrame("goal1")
        if not goal_frame:
            #If we cannot find the name goal1
            #Find anything with the name goal1 in its name 
            #just as a fallback 
            all_frames = self.C.getFrameNames()
            goals = [f for f in all_frames if "goal" in f]
            if goals:
                goal_frame = self.C.getFrame(goals[0])
                print(f"Warning: 'goal1' not found. Using '{goals[0]}' as goal.")
            else:
                raise ValueError("Frame 'goal1' not found in .g file!")
        
        goal_3d = goal_frame.getPosition()
        self.goal_pos = np.array([goal_3d[0], goal_3d[1]])
        
        #define self bounds
        #these are approx
        #Can be adjusted 
        self.bounds = [-4, 4, -4, 4] 

        #Init the graph
        self.start_node = GraphNode(self.start_pos)
        self.goal_node = GraphNode(self.goal_pos)
        self.nodes = [self.start_node, self.goal_node]
        self.tie_breaker = itertools.count()

    def set_robot(self, pos_2d):
        #Moves the robot frame directly 

        #construct 3d position 
        #keeping the original z height the same 

        new_pos_3d = [pos_2d[0], pos_2d[1], self.robot_z]
        
        # Set the frame position and update collisions
        self.C.getFrame(self.robot_name).setPosition(new_pos_3d)
        self.C.computeCollisions()

    def is_removable(self, frame_name):
        #Detects if an object is removable 
        #Based on the g file logic 

        frame = self.C.getFrame(frame_name)
        if not frame: return False
        
        #ADDITIONAL CHECK
        #look for movable in the information string 

        info = frame.info()
        if "movable" in str(info):
            return True
            
        #Name fallback 
        #CHECK THE PREFİXES

        if frame_name.startswith("obj") or frame_name.startswith("obs"):
            return True
            
        return False

    def check_collisions(self):

        #queries the simulation for collisions at the current state
        #Returns either hard collision 
        #or the set of soft obstacle names 

        collisions = self.C.getCollisions()
        is_hard = False
        soft_hits = set()

        for col in collisions:
            # [frameA, frameB, distance]
            f1, f2, dist = col[0], col[1], col[2]

            # rai should return - distance for penetration
            if dist < 0:
                other = f1 if f1 != self.robot_name else f2
                

                #ignore the self collisions 
                if other == self.robot_name: continue
                
                if self.is_removable(other):
                    soft_hits.add(other)
                else:
                    #If we hit a wall or a static object
                    #that is a hard collision 
                    is_hard = True
                    break 
        
        return is_hard, soft_hits

    def get_edge_data(self, from_pos, to_pos, step_size=0.1):

        #Interpolate along the edge to find all obstacles and calculate the cost 

        d = np.linalg.norm(from_pos - to_pos)
        steps = int(d / step_size) + 1
        
        collected_soft = set()
        
        for i in range(steps + 1):
            alpha = i / steps
            interp_pos = (1 - alpha) * from_pos + alpha * to_pos
            
            self.set_robot(interp_pos)
            is_hard, soft_hits = self.check_collisions()
            
            if is_hard:
                return float('inf'), {}
            
            collected_soft.update(soft_hits)

        #The cost function for the path is calculated herer
        # MCR Cost Function = distance + (number of obstacles * penalty)

        cost = d + (len(collected_soft) * self.penalty)
        
        return cost, {obs: 1 for obs in collected_soft}

    def plan(self, max_iters=2000, step_size=0.5, conn_radius=1.5):
        print(f"Starting MCR Plan... ({max_iters} iterations)")
        
        #FIRST DO THE PRM SAMPLING
        #this is basically rrt like sampling for nodes 

        for _ in range(max_iters):
            #Sampling with a goal bias 
            if random.random() < 0.1:
                q_rand = self.goal_pos
            else:
                q_rand = np.array([
                    random.uniform(self.bounds[0], self.bounds[1]),
                    random.uniform(self.bounds[2], self.bounds[3])
                ])

            #find the nearest node 
            q_near = min(self.nodes, key=lambda n: np.linalg.norm(n.pos - q_rand))
            
            #Steer towards the node 
            #limit extension to the given step size 
            vec = q_rand - q_near.pos
            dist = np.linalg.norm(vec)
            if dist > step_size:
                vec = (vec / dist) * step_size
            q_new = q_near.pos + vec
            
            #Optimization
            #Check point validity before we check for edges 
            #Edge checking is very expensive 

            self.set_robot(q_new)
            is_hard, _ = self.check_collisions()
            if is_hard: continue

            #add new node 
            new_node = GraphNode(q_new)
            self.nodes.append(new_node)
            
            #connect the neighbors inside of the connection radius of the new node 
            neighbors = [n for n in self.nodes if np.linalg.norm(n.pos - q_new) <= conn_radius]
            for n in neighbors:
                if n == new_node: continue
                
                #check edge
                cost, disps = self.get_edge_data(n.pos, new_node.pos)
                if cost != float('inf'):
                    n.neighbors.append((new_node, cost, disps))
                    new_node.neighbors.append((n, cost, disps))

        print(f"Graph built with {len(self.nodes)} nodes")
        return self.run_astar()

    def run_astar(self):
        #Here we use euclidian distance 
        start_h = np.linalg.norm(self.start_pos - self.goal_pos)
        
        #We will use a prio queue 
        #f_score, g_score, tie_breaker, current_node, parent, constr_on_edge
        pq = [(start_h, 0, next(self.tie_breaker), self.start_node, None, {})]
        
        visited_costs = {}
        came_from = {} 

        while pq:
            f, g, _, current, parent, edge_disps = heapq.heappop(pq)

            if current in visited_costs and visited_costs[current] <= g:
                continue
            visited_costs[current] = g
            came_from[current] = (parent, edge_disps)

            # Goal Check 
            #only within 0.2 units 
            if np.linalg.norm(current.pos - self.goal_pos) < 0.2:
                print(f"Goal reached! Path Cost: {g:.4f}")
                return self.reconstruct_path(came_from, current, g)

            for neighbor, edge_cost, n_disps in current.neighbors:
                new_g = g + edge_cost
                if neighbor not in visited_costs or new_g < visited_costs[neighbor]:
                    h = np.linalg.norm(neighbor.pos - self.goal_pos)
                    heapq.heappush(pq, (new_g + h, new_g, next(self.tie_breaker), neighbor, current, n_disps))

        return None

    def reconstruct_path(self, came_from, current, total_cost):
        path = []
        all_disps = set()
        
        # Backtrack to construct the path 
        while current:
            path.append(current.pos)
            if current in came_from:
                parent, disps = came_from[current]
                all_disps.update(disps.keys()) # Collect unique objects that must be removed according to the path 
                current = parent
            else:
                current = None
                
        path.reverse()
        return {
            "path": path,
            "total_cost": total_cost,
            "removed_constraints": list(all_disps),
            "num_removals": len(all_disps)
        }

if __name__ == "__main__":
    #CONFIGURATION

    G_FILE = "config/slot/slot.g"
    
    try:
        # Init the planner 
        planner = RAIMCRPlanner(G_FILE, penalty=100.0)
        
        #Visualize
        print("Initial Configuration:")
        planner.C.view()
        time.sleep(1)

        
        result = planner.plan(max_iters=2000, step_size=0.4)

        if result:
                print("\n" + "="*40)
                print(f"SUCCESS: Path Found")
                print(f"Objects that must be removed: {result['num_removals']}")
                print(f"List of Objects: {result['removed_constraints']}")
                print("="*40)

                #Visualize the path 
                #Use spheres for this 
                for i, p in enumerate(result["path"]):
                    planner.C.addFrame(f"path_node_{i}") \
                        .setShape(ry.ST.sphere, [0.05]) \
                        .setPosition([p[0], p[1], 0.1]) \
                        .setColor([0, 1, 0]) # Green dots

                #highlight the removed objects 
                #turn them red 
                for obj_name in result['removed_constraints']:
                    f = planner.C.getFrame(obj_name)
                    if f: f.setColor([1, 0, 0]) 
                    
                planner.C.view()
                
                print("Animating Robot...")
                for p in result["path"]:
                    planner.set_robot(p)
                    time.sleep(0.05)
                    
        else:
            print("No path found.")
                
        time.sleep(2)
        
    except FileNotFoundError:
        print(f"Error: The file '{G_FILE}' was not found.")
        print("Please check that the path is correct relative to where you are running the script.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")