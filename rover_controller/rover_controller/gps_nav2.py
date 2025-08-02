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

    # # convert from radians to degrees
    # roll_deg = math.degrees(roll)
    # pitch_deg = math.degrees(pitch)
    # yaw_deg = math.degrees(yaw)

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
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_yaw = 0.0
        self.is_first_gps_msg = True
        self.origin_lat = 0.0
        self.origin_lon = 0.0
        
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
        if self.is_first_gps_msg:
            self.origin_lat = self.current_lat
            self.origin_lon = self.current_lon
            self.is_first_gps_msg = False
            self.get_logger().info(f"Origin set at LAT: {self.origin_lat}, LON: {self.origin_lon}")
            self.get_logger().info(f"Navigating to Target: LAT={self.target_lat}, LON={self.target_lon}")

    def imu_callback(self, msg):
        self.current_yaw = euler_from_quaternion(msg.orientation)[-1]

    def convert_gps_to_xy(self, lat, lon):
        R = 6378137.0  # Earth radius in meters
        x = R * (lon - self.origin_lon) * math.cos(math.radians(self.origin_lat))
        y = R * (lat - self.origin_lat)
        return round(x, 6), round(y, 6)

    def navigation_loop(self):
        # Wait until the origin is set from the first GPS message
        if self.is_first_gps_msg:
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

            twist_msg.linear.x = min(0.5, linear_velocity) # Cap max speed
            twist_msg.angular.z = angular_velocity
        else:
            self.get_logger().info("Goal Reached!")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            # Optional: shutdown node once goal is reached
            rclpy.shutdown()
        
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    gps_navigator = GpsNavigator()
    rclpy.spin(gps_navigator)
    gps_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()