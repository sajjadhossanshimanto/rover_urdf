import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import math


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

def calculate_bearing(current_lat, current_lon, target_lat, target_lon):
    """
    Calculate the bearing (initial heading) from a current GPS coordinate to a target GPS coordinate.
    Result is in radians.
    """
    current_lat = math.radians(current_lat)
    current_lon = math.radians(current_lon)
    target_lat = math.radians(target_lat)
    target_lon = math.radians(target_lon)

    dLon = target_lon - current_lon

    y = math.sin(dLon) * math.cos(target_lat)
    x = math.cos(current_lat) * math.sin(target_lat) - \
        math.sin(current_lat) * math.cos(target_lat) * math.cos(dLon)

    bearing = math.atan2(y, x)
    return bearing

def calculate_distance(current_lat, current_lon, target_lat, target_lon):
    """
    Calculate the great-circle distance between two points
    on the earth (specified in decimal degrees).
    Result is in meters.
    """
    # Radius of earth in kilometers
    R = 6371.0

    lat1 = math.radians(current_lat)
    lon1 = math.radians(current_lon)
    lat2 = math.radians(target_lat)
    lon2 = math.radians(target_lon)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c * 1000 # convert to meters
    return distance

class State:
    ideal = "IDEAL"
    target_received = "GOAL_SET"
    rotating = "ROTATING"
    rotaion_completed = "ROTATION_COMPLETED"
    running = "MOVING_FORWARD"
    goal_reached = "GOAL_REACHED"


class GpsNavigator(Node):
    """
    A ROS2 node for navigating a robot to a target GPS coordinate.
    """
    def __init__(self):
        super().__init__('gps_navigator')

        # --- Parameters ---
        self.declare_parameter('target_latitude', 0.0)
        self.declare_parameter('target_longitude', 0.0)
        self.declare_parameter('linear_speed', 0.5) # m/s
        self.declare_parameter('angular_speed', 0.3) # rad/s
        self.declare_parameter('distance_tolerance', 1.0) # meters
        self.declare_parameter('angle_tolerance', 0.05) # radians

        self.target_lat = self.get_parameter('target_latitude').value
        self.target_lon = self.get_parameter('target_longitude').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        # --- State Variables ---
        self.current_lat = None
        self.current_lon = None
        self.current_yaw = None
        self.last_state = State.ideal # Can be IDLE, ROTATING, MOVING_FORWARD, GOAL_REACHED

        # --- Subscribers ---
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps', # Make sure this topic is correct
            self.gps_callback,
            10)
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu', # Make sure this topic is correct
            self.imu_callback,
            10)

        # --- Publisher ---
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Control Loop Timer ---
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"GPS Navigator Node Started. Target: LAT={self.target_lat}, LON={self.target_lon}")
        self.get_logger().info("Waiting for first GPS and IMU messages...")


    def gps_callback(self, msg):
        """Callback for the GPS sensor."""
        if self.current_lat is None:
            self.get_logger().info("First GPS message received.")
        self.current_lat = round(msg.latitude, 6)# 6 digit = 11cm precition
        self.current_lon = round(msg.longitude, 6)

    def imu_callback(self, msg):
        """Callback for the IMU sensor."""
        if self.current_yaw is None:
            self.get_logger().info("First IMU message received.")
        self.current_yaw = euler_from_quaternion(msg.orientation)[-1]

    def control_loop(self):
        """Main control loop for state-based navigation."""
        # Wait until we have initial sensor readings
        if self.current_lat is None or self.current_yaw is None:
            self.get_logger().info("waiting for sensor data.")
            return

        # Start navigation if we are idle and have a valid target
        if self.last_state == State.ideal:
            if self.target_lat != 0.0 and self.target_lon != 0.0:
                self.get_logger().info("Valid target and sensor data. Starting navigation.")
                self.last_state = State.target_received
            else:
                self.get_logger().warn("Target coordinates are not set. Remaining in IDLE state.")
            return

        if self.last_state == State.target_received:
            bearing_to_target = calculate_bearing(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
            angle_diff = self.normalize_angle(bearing_to_target - self.current_yaw)
            
            if abs(angle_diff) > self.angle_tolerance:
                self.rotate_robo(1 if angle_diff > 0 else -1)
            self.last_state = State.rotating
            
        if self.last_state == State.rotating:
            bearing_to_target = calculate_bearing(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
            angle_diff = self.normalize_angle(bearing_to_target - self.current_yaw)
            
            if abs(angle_diff) > self.angle_tolerance:
                self.get_logger().info(f"rotating towards target: {self.current_yaw} -> {bearing_to_target}")
            else:
                # at this point status is rotation completed
                self.stop_robot()
                self.get_logger().info("Rotation complete. Switching to MOVING_FORWARD.")
                self.move_forward()
                self.last_state = State.running
            return 


        # Check if we have reached the goal
        if self.last_state == State.running:
            distance_to_target = calculate_distance(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
            if distance_to_target <= self.distance_tolerance:
                self.stop_robot()
                self.get_logger().info(f"Goal reached! Distance to target: {distance_to_target:.2f}m")
                self.last_state = State.goal_reached
            else:
                self.get_logger().info(f"Moving towards ({self.current_lat}, {self.current_lon}) -> ({self.target_lat}, {self.target_lon})")
            return

        #     # TODO Optional: A simple proportional controller to correct heading while moving
        #     angle_diff_correction = self.normalize_angle(bearing_to_target - self.current_yaw)
        #     if abs(angle_diff_correction) > self.angle_tolerance:
        #          twist_msg.angular.z = 0.1 * angle_diff_correction # Small correction


    def rotate_robo(self, direction=-1):
        twist_msg = Twist()
        twist_msg.angular.z = direction*self.angular_speed

        self.velocity_publisher.publish(twist_msg)

    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
    
        self.velocity_publisher.publish(twist_msg)

    def stop_robot(self):
        """Publishes a zero-velocity Twist message to stop the robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)
        self.get_logger().info("Robot stopped.")

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    gps_navigator = GpsNavigator()
    # gps_navigator.target_lat = -22.986777
    # gps_navigator.target_lon = -43.202501
    try:
        rclpy.spin(gps_navigator)
    except KeyboardInterrupt:
        gps_navigator.get_logger().info("Navigation interrupted by user.")
    finally:
        gps_navigator.stop_robot()
        gps_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
origin: (-22.986687, -43.202501) -> heading north
# north
ros2 run rover_controller gps_nav  --ros-args -p target_latitude:=-22.986597 -p target_longitude:=-43.202501
# south
ros2 run rover_controller gps_nav  --ros-args -p target_latitude:=-22.986777 -p target_longitude:=-43.202501
# east
ros2 run rover_controller gps_nav  --ros-args -p target_latitude:=-22.986687 -p target_longitude:=-43.202403
# west
ros2 run rover_controller gps_nav  --ros-args -p target_latitude:=-22.986687 -p target_longitude:=-43.202599

'''