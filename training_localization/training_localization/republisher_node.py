import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

class RepublisherNode(Node):

    def __init__(self):
        super().__init__('localization_republisher')
        self.measured_IMU = Imu()
        self.gps_fix = NavSatFix()
        self.IMU_ready = False
        self.GPS_ready = False
        

        self.imuPublisher = self.create_publisher(Imu, '/navsat/imu', 10)
        self.gpsPublisher = self.create_publisher(NavSatFix, '/navsat/gps', 10)
        
        #subscribe to wamv sensor data
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            10)
        self.gps_fix_subscriber = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_fix_callback,
            10)

    def imu_callback(self, msg):
        self.measured_IMU = msg
        self.IMU_ready = True
        self.state_estimation()

        
    def gps_fix_callback(self, msg):
        self.gps_fix = msg
        self.GPS_ready = True
        self.state_estimation()
    
    #if all the data is ready, publish it to the ekf and navsattransform nodes
    def state_estimation(self):
        if not self.GPS_ready or not self.IMU_ready:
            return

        self.imuPublisher.publish(self.measured_IMU)
        self.gpsPublisher.publish(self.gps_fix)
        
def main(args=None):
    rclpy.init(args=args)

    continual_EKF = RepublisherNode()

    rclpy.spin(continual_EKF)

    continual_EKF.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
