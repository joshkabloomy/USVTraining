from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
import math
import heapq

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
class AStarCustomPlanner(Planner):
    def __init__(self):
        super().__init()
        self.x_width, self.y__width = 100, 100
        self.obstacle_map = None
    
    class Node:
        def __init__(self, x, y, cost, parent_node):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_node = parent_node
        # comparator function
        def __lt__(self, other):
            return self.val < other.val
    
    def set_obstacle_map(self):
        pass
        obstacle_map = []
        # instantiate the matrix
        for i in range(0, self.x_width):
            obstacle_map[i] = [[False for j in range(0, self.y__width)]]
        
        
        # put true for the box obstacle
        
        small_y, large_y, small_x, large_x = None, None, None, None
        
        for a in self.bounding_boxes.corners: # find the orientation of obstacle
            small_y = min(a.y, small_y) if small_y is not None else a.y
            small_x = min(a.x, small_x) if small_x is not None else a.x
            large_y = min(a.y, large_y) if large_y is not None else a.y
            large_x = min(a.x, large_x) if large_y is not None else a.x
        
        if small_y == large_y:
            for i in range(small_x, large_x + 1):
                obstacle_map[i][small_y] = True
                obstacle_map[i+1][small_y] = True # depth of the bounding box
        else:
            for i in range(small_y, large_y + 1):
                obstacle_map[small_x][i] = True
                obstacle_map[small_x][i+1] = True
                
        self.obstacle_map = obstacle_map
    
    
    # transform from frame to coordinate plane
    def calc_grid_position(self, node):
        return (node.x + self.x_width/2, node.y + self.y_width/2)
    
    # check bounds 
    def verify_node(self, node):
        px, py = self.calc_grid_position(node)
    
    @staticmethod
    def calc_heuristic(node1, node2):
        w = 1.0
        return w * math.hypot(node1.x - node2.x, node1.y - node2.y)
    
    
    # A* algorithm
    def create_plan(self):
        if self.robot_pose is None or self.goal_pose is None:
            return Path()
        path = Path()
        path.header.frame_id = 'map'
        
        start_node = self.Node(self.robot_pose.position.x, self.robot_pose.position.y, 0.0, -1)
        goal_node = self.Node(self.goal_pose.position.x, self.goal_pose.position.y, 0.0, -1)
        
        visited = set()
        path_set = dict()
        #start = Planner.pose_deep_copy(self.robot_pose)
       
        queue = []
        heapq.heappush(queue, start_node)
        
        while True:
            if len(queue) == 0:
                print("Queue is empty")
                break
            # pops out of heap
            current = heapq.heappop(queue)
            
            if current.position.x == self.goal_pose.position.x and current.position.y == self.goal_pose.position.y:
                
                pass
            
            node_left = self.Node(current.x - 1, current.y, current.cost + 1 + self.calc_heuristic(current, goal_node), current)
            node_right = self.Node(current.x + 1, current.y, current.cost + 1 + self.calc_heuristic(current, goal_node), current)
            node_forward = self.Node(current.x, current.y + 1, current.cost + 1 + self.calc_heuristic(current, goal_node), current)
            node_back = self.Node(current.x, current.y - 1, current.cost + 1 + self.calc_heuristic(current, goal_node), current)
            
            