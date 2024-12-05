import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math
import tf_transformations

class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')
        self.marker_pub = self.create_publisher(Marker, 'line_marker', 10)
        self.waypoints = [[1.0, 3.0], [3.0, 4.0], [3.0, 5.0]]
        self.next_waypoint = 0
        self.radius = 0.2
        self.move_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.sub_callback, 10)
        self.current_pose = Pose()
        self.timer = self.create_timer(2, self.distance_calculate)
        
    def sub_callback(self, received_msg:Odometry):
        #self.get_logger().info('Publishing: x: {} y: {} z: {}'.format(received_msg.pose.pose.position.x, received_msg.pose.pose.position.y, received_msg.pose.pose.position.z))
        self.current_pose = received_msg.pose.pose
    
    def distance_calculate(self):
        
        self.draw_waypoints()
        
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z
        
        delta_x = self.waypoints[self.next_waypoint][0] - x
        delta_y = self.waypoints[self.next_waypoint][1] - y
        
        
        distance = math.sqrt(delta_x**2 + delta_y**2)
        print(self.get_logger().info('Distance: {}'.format(distance)))
        
        if distance < self.radius:
            if self.next_waypoint == len(self.waypoints)-1:
                self.next_waypoint = 0
                
                
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
        angle = math.atan2(delta_y, delta_x)
        theta = angle - yaw  
        
        
        cmd_msg = Twist()

        # Set a default forward speed
        cmd_msg.linear.x = 0.1     
        # Turn
        cmd_msg.angular.z = theta
        self.move_publisher.publish(cmd_msg)
        
    def draw_waypoints(self):
        marker = Marker()
        # Set up the marker properties
        marker.header.frame_id = 'odom'  # Change to your frame of reference
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lines'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set the scale and color of the line
        marker.scale.x = 0.02  # Line width
        marker.color.r = 1.0    # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0    # Full opacity
        
        for p in self.waypoints:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            marker.points.append(point)
        
        self.marker_pub.publish(marker)

        
def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()