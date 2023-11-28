from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
import math
import heapq
import sys 

###

# For trainees:
# Implement the CustomPlanner class at the bottom of the file
# Feel free to change other files as well, though you shouldn't need to make a lot of changes

###

class Planner:

    def __init__(self):
        self.robot_pose:Pose = None
        self.goal_pose:Pose = None
        self.bounding_boxes = None

    # Helper method to create a deep copy of a Pose
    def pose_deep_copy(pose:Pose):
        copy = Pose()
        copy.position.x = pose.position.x
        copy.position.y = pose.position.y
        copy.position.z = pose.position.z
        copy.orientation.x = pose.orientation.x
        copy.orientation.y = pose.orientation.y
        copy.orientation.z = pose.orientation.z
        copy.orientation.w = pose.orientation.w

        return copy

class StraightPlanner(Planner):

    def create_plan(self):
        if self.robot_pose is None or self.goal_pose is None:
            return Path()
        
        if self.goal_pose.position.x - self.robot_pose.position.x > 0:
            x_dir = 1
        else:
            x_dir = -1
        
        if self.goal_pose.position.y - self.robot_pose.position.y > 0:
            y_dir = 1
        else:
            y_dir = -1

        path = Path()
        path.header.frame_id = 'map'

        if self.goal_pose.position.x - self.robot_pose.position.x == 0.0:
            theta = .5 * math.pi
        else:
            theta = math.atan(abs(
                (self.goal_pose.position.y - self.robot_pose.position.y) / (self.goal_pose.position.x - self.robot_pose.position.x)
            ))

        curr_pose = Planner.pose_deep_copy(self.robot_pose)

        while (
            (abs(curr_pose.position.x - self.robot_pose.position.x) 
                < abs(self.goal_pose.position.x - self.robot_pose.position.x)) and 
            (abs(curr_pose.position.y - self.robot_pose.position.y) 
                < abs(self.goal_pose.position.y - self.robot_pose.position.y))
        ):
            path.poses.append(PoseStamped(pose=Planner.pose_deep_copy(curr_pose)))

            curr_pose.position.x += math.cos(theta) * 0.2 * x_dir
            curr_pose.position.y += math.sin(theta) * 0.2 * y_dir

        path.poses.append(PoseStamped(pose=self.goal_pose))

        return path


# For you to implement!
class CustomPlanner(Planner):
    def __init__(self):
        self.x_width, self.y_width = 50,50
        self.obstacle_map = None
    
    class Node:
        def __init__(self, x, y, cost, parent_node):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_node = parent_node
        # comparator function
        def __lt__(self, other):
            return self.cost < other.cost
        # equals function
        def __eq__(self, other):
            return self.x == other.x and self.y == other.y
        
        def __hash__(self):
        # Hash the combination of x and y
            return hash((self.x, self.y))
        
        def __repr__(self):
            return str(str(self.x) + " "  + str(self.y) +  " ")
                    #    + str(self.cost) +  " " + str(self.parent_node))

    
    def set_obstacle_map(self):
        obstacle_map = []
        # instantiate the matrix
        
        obstacle_map = [[False for a in range(self.y_width)] for b in range(self.x_width)]
        
        # put true for the box obstacle
        
        # self.node.get_logger().info(str(self.bounding_boxes))
        for a in self.bounding_boxes.boxes: # find the orientation of obstacle
            small_y, small_x = 100000,100000
            large_y, large_x = -100000,-100000
            
            #self.node.get_logger().info(str(small_x) + " " + str(large_x) + " "+ str(small_y) + " " + str(large_y) + " ")
            for b in a.corners:
                # self.node.get_logger().info("corners  " + str(b))
                small_y = round(min(self.calc_xy_index(b.y, self.y_width), small_y) )
                small_x = round(min(self.calc_xy_index(b.x, self.x_width), small_x) )
                large_y = round(max(self.calc_xy_index(b.y, self.y_width), large_y) )
                large_x = round(max(self.calc_xy_index(b.x, self.x_width), large_x))
            # self.node.get_logger().info("BREAK ---- ")
            exceed_size = False
            if (large_y >= self.y_width or large_x >= self.x_width or small_x<0 or small_y<0):
                exceed_size = True
            self.node.get_logger().info(str(small_x) + " " + str(large_x) + " "+ str(small_y) + " " + str(large_y) + " ")
                
            # update the matrix
            if not exceed_size and small_y == large_y:
                self.node.get_logger().info("small_X: " + str(small_x) )
                self.node.get_logger().info("large_x: " + str(large_x) )
                for i in range(small_x, large_x + 1):
                    obstacle_map[i][small_y] = True
                    self.node.get_logger().info("visited_x_of y: " + str(obstacle_map[24][25]) )
                    if (small_y + 1 < self.y_width):
                        obstacle_map[i][small_y + 1] = True # depth of the bounding box
                if (small_x - 1 >= 0) :
                    obstacle_map[small_x - 1][small_y] = True
                    #self.node.get_logger().info("small_x " + str(small_x - 1) )
                if (large_x + 1 < self.x_width):
                    obstacle_map[large_x + 1][small_y] = True
            
            elif not exceed_size and small_x == large_x:
                self.node.get_logger().info("small_y: " + str(small_y) )
                self.node.get_logger().info("large_y: " + str(large_y) )
                for i in range(small_y, large_y + 1):
                    obstacle_map[small_x][i] = True
                    self.node.get_logger().info("visited_x_of y: " + str(obstacle_map[24][25]))
                    self.node.get_logger().info("visited_x_of y: " + str(small_x)  + " " + str(i))
                    if (small_x + 1 < self.x_width):
                        obstacle_map[small_x + 1][i] = True
                if (small_y - 1 >= 0) :
                    obstacle_map[small_x][small_y - 1] = True
                if (small_y + 1 < self.y_width):
                    obstacle_map[small_x][large_y + 1] = True
            
            else :
                self.node.get_logger().info("Obstacle updating error: Too big")
                #self.node.get_logger().info(str(a.corners))
                #self.node.get_logger().info("obstacle: " + str(small_x) + " " + str(large_x) + " "+ str(small_y) + " " + str(large_y) + " ")
                
        
        # for i in range(0, self.x_width):
        #     self.node.get_logger().info(str(obstacle_map[i]))
        self.obstacle_map = obstacle_map
    
    
    # transform from coordinate plane to matrix coordinates
    def calc_grid_position(self, node):
        return (round(node.x + self.x_width/2), round(node.y + self.y_width/2))
    # calc a specific x/y coordiante
    def calc_xy_index(self, position, size):
        return round(position + size/2)
    #specific string to access a certain node in (x,y) coord
    def calc_grid_string(self, node):
        return str(node.x) + " " + str(node.y)
    
    # check bounds 
    def verify_node(self, node):
        px, py = self.calc_grid_position(node)
        #self.node.get_logger().info("Check bounds" + str(px) + " " + str(py))
        #self.node.get_logger().info("Size of Matrix" + str(len(self.obstacle_map)) + " " + str(len(self.obstacle_map[0])))
        if px < 0:
            return False
        elif py < 0:
            return False
        elif px >= self.x_width:
            return False
        elif py >= self.y_width:
            return False
        #self.node.get_logger().info(str(self.obstacle_map[px][py]))
        # check if hits obstacle
        if self.obstacle_map[px][py] == True:
            self.node.get_logger().info("checker: " + str(px) + " " + str(py))
            return False
        
        return True
    
    @staticmethod
    def calc_heuristic(node1, node2):
        w = 1.0
        return w * math.hypot(node1.x - node2.x, node1.y - node2.y)
    #backtrack the path
    def return_path(self, found_node, start_node, path):
        
        curr_pose = Pose()
        temp_node = found_node
        while self.calc_grid_string(temp_node) != self.calc_grid_string(start_node):
            curr_pose = Pose()
            curr_pose.position.x = float(temp_node.x)
            curr_pose.position.y = float(temp_node.y)
            path.poses.append(PoseStamped(pose=Planner.pose_deep_copy(curr_pose)))
            self.node.get_logger().info(self.calc_grid_string(temp_node))
            temp_node = temp_node.parent_node
            
        curr_pose = Pose()
        curr_pose.position.x = float(temp_node.x)
        curr_pose.position.y = float(temp_node.y)
        path.poses.append(PoseStamped(pose=Planner.pose_deep_copy(curr_pose)))
        path.poses.reverse()
        return path
        
        
    
    # A* algorithm
    def create_plan(self):
        if self.robot_pose is None or self.goal_pose is None:
            return Path()
        path = Path()
        path.header.frame_id = 'map'
        
        self.set_obstacle_map()
        # self.node.get_logger().info(str(self.robot_pose.position.x))
        start_x_val = round(self.robot_pose.position.x)
        # self.node.get_logger().info(str(start_x_val))
        start_y_val = round(self.robot_pose.position.y)
        start_node = self.Node(start_x_val, 
                               start_y_val, 0.0, -1)
        goal_node = self.Node(round(self.goal_pose.position.x),
                               round(self.goal_pose.position.y), 0.0, -1)
        
        self.node.get_logger().info("Goal position: " + self.calc_grid_string(goal_node))
        if not self.verify_node(goal_node):
            self.node.get_logger().info("Cannot find path to obstacle!")
            return Path()
        #start = Planner.pose_deep_copy(self.robot_pose)
        path_value = dict()
        path_value[self.calc_grid_string(start_node)] = 0
        
        visited = set() # we do not to revisit a node twice 
        
    
        queue = []
        heapq.heappush(queue, start_node)
        counter = 0
        while True:
            counter += 1
            #self.node.get_logger().info(str(len(queue) == 0))
            if len(queue) == 0:
                self.node.get_logger().info("empty queue")
                break
            # pops out of heap
            #self.node.get_logger().info(str(list(queue)))
            current = heapq.heappop(queue)
            # self.node.get_logger().info("queue length " + str(len(queue)))
            self.node.get_logger().info("position " + self.calc_grid_string(current))
            
            visited.add(self.calc_grid_string(current))
            #self.node.get_logger().info("visited: " + str(visited))
            
            if current.x == goal_node.x and current.y == goal_node.y:
                self.node.get_logger().info("Found the goal!!")
                self.node.get_logger().info("Nodes Visited: " + str(counter))
                self.return_path(current, start_node, path)
                return path
                
            
            node_left = self.Node(current.x - 1, current.y, current.cost + 1 + self.calc_heuristic(current, goal_node), current)
            node_right = self.Node(current.x + 1, current.y, current.cost + 1 + self.calc_heuristic(current, goal_node), current)
            node_forward = self.Node(current.x, current.y + 1, current.cost + 1 + self.calc_heuristic(current, goal_node), current)
            node_back = self.Node(current.x, current.y - 1, current.cost + 1 + self.calc_heuristic(current, goal_node), current)
            
            if not self.verify_node(node_left) or self.calc_grid_string(node_left) in visited:
                node_left = None
            if not self.verify_node(node_right) or self.calc_grid_string(node_right) in visited:
                node_right = None
            if not self.verify_node(node_forward) or self.calc_grid_string(node_forward) in visited:
                node_forward = None
            if not self.verify_node(node_back) or self.calc_grid_string(node_back) in visited:
                node_back = None
                
            # if self.calc_grid_position(node_left) in path_value:
            #     continue
            
            # Now checking & updating values in queue
        
        
            if node_left is not None and node_left not in queue:
                heapq.heappush(queue, node_left)
            elif node_left is not None:
                # updates the node value
                for i in range(len(queue)):
                        if node_left.x == queue[i].x and node_left.y == queue[i].y:
                            if queue[i].cost > node_left.cost:
                                queue[i] = node_left
                                heapq.heapify(queue)
            
            if node_right is not None and node_right not in queue:
                heapq.heappush(queue, node_right)
            elif node_right is not None:
                # updates the node value
                for i in range(len(queue)):
                        if node_right.x == queue[i].x and node_right.y == queue[i].y:
                            if queue[i].cost > node_right.cost:
                                queue[i] = node_right
                                heapq.heapify(queue)
            
            if node_forward is not None and node_forward not in queue:
                heapq.heappush(queue, node_forward)
            elif node_forward is not None:
                # updates the node value
                for i in range(len(queue)):
                        if node_forward.x == queue[i].x and node_forward.y == queue[i].y:
                            if queue[i].cost > node_forward.cost:
                                queue[i] = node_forward
                                heapq.heapify(queue)
            
            if node_back is not None and node_back not in queue:
                heapq.heappush(queue, node_back)
            elif node_back is not None:
                # updates the node value
                for i in range(len(queue)):
                        if node_back.x == queue[i].x and node_back.y == queue[i].y:
                            if queue[i].cost > node_back.cost:
                                queue[i] = node_back
                                heapq.heapify(queue)
            
                    
                    
                    
                    
            