import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry

class TestNavigationNode(Node):

    def __init__(self):
        super().__init__('test_navigation')

        self.declare_parameters(namespace='', parameters=[
            ('planner', 'STRAIGHT_LINE')
        ])

        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)
        
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', 
            self.goal_callback, 10)

        self.robot_pose:Pose = None
        self.goal:Pose = None

    def odom_callback(self, msg:Odometry):
        self.robot_pose = msg.pose.pose
    
    def goal_callback(self, msg:PoseStamped):
        self.goal = msg.pose

def main(args=None):
    rclpy.init(args=args)

    node = TestNavigationNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()