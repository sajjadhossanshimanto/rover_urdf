import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist


def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return round(roll, 6), round(pitch, 6), round(yaw, 6)

class GpsNavigator(Node):
    def __init__(self):
        super().__init__('gps_navigator_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # --- Declare and Get Parameters ---
        self.declare_parameter('target_lat', -22.986777) # Example Target Latitude
        self.declare_parameter('target_lon', -43.202501) # Example Target Longitude
        self.declare_parameter('k_p_angular', 1.5)
        self.declare_parameter('k_p_linear', 0.1)
        self.declare_parameter('distance_tolerance', 1.5) # meters

        self.target_lat = self.get_parameter('target_lat').value
        self.target_lon = self.get_parameter('target_lon').value
        self.k_p_angular = self.get_parameter('k_p_angular').value
        self.k_p_linear = self.get_parameter('k_p_linear').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value

        # --- State Variables ---
        self.current_lat = None
        self.current_lon = None
        self.current_yaw = None
        self.origin_lat = None
        self.origin_lon = None
        
        # --- ROS Publishers & Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(NavSatFix, '/gps', self.gps_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # --- Main Control Loop Timer ---
        self.timer = self.create_timer(0.1, self.navigation_loop) # 10 Hz loop
        self.get_logger().info("GPS Navigator node started. Waiting for sensor data...")


    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = self.current_lat
            self.origin_lon = self.current_lon
            self.get_logger().info(f"Origin set at LAT: {self.origin_lat}, LON: {self.origin_lon}")
            self.get_logger().info(f"Navigating to Target: LAT={self.target_lat}, LON={self.target_lon}")

    def imu_callback(self, msg):
        self.current_yaw = euler_from_quaternion(msg.orientation)[-1]

    def convert_gps_to_xy(self, lat, lon):
        R = 378137.0  # Earth radius in meters
        x = R * (lon - self.origin_lon) * math.cos(math.radians(self.origin_lat))
        y = R * (lat - self.origin_lat)
        return round(x, 6), round(y, 6)

    def navigation_loop(self):
        # Wait until the origin is set from the first GPS message
        if None in (self.origin_lat, self.origin_lon, self.current_yaw):
            return

        current_x, current_y = self.convert_gps_to_xy(self.current_lat, self.current_lon)
        target_x, target_y = self.convert_gps_to_xy(self.target_lat, self.target_lon)
        self.get_logger().debug(f"({current_x}, {current_y}) -> ({target_x}, {target_y})")

        distance_to_goal = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        twist_msg = Twist()

        if distance_to_goal > self.distance_tolerance:
            angle_to_goal = math.atan2(target_y - current_y, target_x - current_x)
            angle_error = angle_to_goal - self.current_yaw
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            angular_velocity = self.k_p_angular * angle_error
            
            linear_velocity = 0.0
            if abs(angle_error) < math.pi / 4: # 45 degrees
                linear_velocity = self.k_p_linear * distance_to_goal

            twist_msg.linear.x = -min(0.5, linear_velocity) # Cap max speed
            twist_msg.angular.z = angular_velocity
        else:
            self.get_logger().info("Goal Reached!")
            self.stop_robot()
            # shutdown node once goal is reached
            rclpy.shutdown()
        
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        """Publishes a zero-velocity Twist message to stop the robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("Robot stopped.")

def main(args=None):
    rclpy.init(args=args)
    gps_navigator = GpsNavigator()
    rclpy.spin(gps_navigator)
    gps_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
# north
ros2 run rover_controller gps_nav2  --ros-args -p target_lat:=-22.986597 -p target_lon:=-43.202501
# south
ros2 run rover_controller gps_nav2  --ros-args -p target_lat:=-22.986777 -p target_lon:=-43.202501
# east
ros2 run rover_controller gps_nav2  --ros-args -p target_lat:=-22.986687 -p target_lon:=-43.202403
# west
ros2 run rover_controller gps_nav2  --ros-args -p target_lat:=-22.986687 -p target_lon:=-43.202599
'''