import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Driver(Node):
    
    def __init__(self):
        super().__init__('driver_node')
        self.subscriber_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def scan_callback(self, received_msg:LaserScan):
        # Initialize flags for obstacles
        positive_obstacle = False
        negative_obstacle = False
        
        # Define a threshold distance for detecting obstacles
        obstacle_distance_threshold = 0.5  # in meters

        # Iterate through LIDAR range data
        for i in range(0, len(received_msg.ranges)):
            lidar_range = received_msg.ranges[i]
            
            # Skip invalid range values
            if lidar_range > received_msg.range_max or lidar_range < received_msg.range_min:
                continue

            # Calculate the angle for each range value
            angle = received_msg.angle_min + i * received_msg.angle_increment
            
            # Detect obstacles in the positive angle range (right side)
            if angle > 0 and lidar_range < obstacle_distance_threshold:
                positive_obstacle = True
            
            # Detect obstacles in the negative angle range (left side)
            if angle < 0 and lidar_range < obstacle_distance_threshold:
                negative_obstacle = True

        # Create a Twist message for robot movement
        cmd_msg = Twist()

        # Set a default forward speed
        cmd_msg.linear.x = 0.1
        
        # Adjust angular velocity based on obstacles detected
        if positive_obstacle:
            cmd_msg.angular.z = 0.5  # Turn left if an obstacle is detected on the right (positive angles)
        elif negative_obstacle:
            cmd_msg.angular.z = -0.5  # Turn right if an obstacle is detected on the left (negative angles)
        else:
            cmd_msg.angular.z = 0.0  # Move straight if no obstacles are detected

        # Publish the twist message to control robot movement
        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    
    rclpy.init(args=args)
    driver_node = Driver()
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
