import math
import time
from enum import Enum
from dataclasses import dataclass
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy


@dataclass
class ScanSectors:
    """Container for processed scan data by sector."""
    front: list
    left: list
    right: list
    
    def get_safe_distance(self, sector):
        """Safely get minimum distance in a sector with fallback."""
        if not sector:
            return float('inf')
        return min(sector)
    
    def get_average_distance(self, sector):
        """Calculate average distance with filtering."""
        if not sector:
            return float('inf')
        return sum(sector) / len(sector)


class RobotState(Enum):
    """State machine for robust obstacle avoidance."""
    MOVING_FORWARD = 1
    OBSTACLE_DETECTED = 2
    TURNING = 3
    BACKING_UP = 4
    STUCK = 5


class ObstacleAvoider(Node):
    """
    Robust obstacle avoidance controller with state machine and hysteresis.
    
    IMPROVEMENTS:
    - State machine prevents jittery behavior
    - Hysteresis prevents oscillation around thresholds
    - Velocity ramping for smoother motion
    - Configurable safety margins and turning strategies
    - Min readings validation
    - Recovery behaviors for stuck situations
    """
    
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Subscribers and Publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Safety parameters (configurable)
        self.declare_parameter('safe_distance', 1.0)
        self.declare_parameter('critical_distance', 0.5)
        self.declare_parameter('min_sensor_range', 0.3)
        self.declare_parameter('forward_speed', 0.3)
        self.declare_parameter('turn_speed', 1.0)
        self.declare_parameter('backup_speed', -0.2)
        self.declare_parameter('min_sector_readings', 5)
        
        # Get parameters
        self.safe_distance = self.get_parameter('safe_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.min_sensor_range = self.get_parameter('min_sensor_range').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.backup_speed = self.get_parameter('backup_speed').value
        self.min_sector_readings = self.get_parameter('min_sector_readings').value
        
        # State management
        self.current_state = RobotState.MOVING_FORWARD
        self.last_safe_state_time = time.time()
        self.stuck_threshold = 2.0  # seconds without progress
        self.state_transition_damping = 0.3  # seconds to prevent oscillation
        self.last_state_change = time.time()
        
        # Velocity smoothing
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.velocity_ramp_rate = 2.0  # m/s² acceleration limit
        
        # Sector angles (in radians)
        self.FRONT_ANGLE = 0.35  # ±20°
        self.SIDE_ANGLE = 2.2    # ~126°
        
        self.get_logger().info("ObstacleAvoider initialized with robust state machine")
    
    def _clean_scan_data(self, ranges, angle_min, angle_increment):
        """Filter invalid readings and return cleaned data."""
        cleaned = []
        for i, r in enumerate(ranges):
            # Skip invalid readings
            if not math.isfinite(r):
                continue
            # Skip very close readings (robot self-collision detection)
            if r < self.min_sensor_range:
                continue
            cleaned.append(r)
        return cleaned
    
    def _segment_scan(self, msg):
        """
        Divide scan into front, left, right sectors.
        Returns ScanSectors with validated data.
        """
        front_ranges = []
        left_ranges = []
        right_ranges = []
        
        # Clean data first
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < self.min_sensor_range:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            
            # Segment by angle
            if -self.FRONT_ANGLE < angle < self.FRONT_ANGLE:
                front_ranges.append(r)
            elif self.FRONT_ANGLE <= angle < self.SIDE_ANGLE:
                left_ranges.append(r)
            elif -self.SIDE_ANGLE < angle <= -self.FRONT_ANGLE:
                right_ranges.append(r)
        
        return ScanSectors(front_ranges, left_ranges, right_ranges)
    
    def _check_state_validity(self, sectors):
        """Validate we have enough readings in critical sectors."""
        if len(sectors.front) < self.min_sector_readings:
            self.get_logger().warn(
                f"Insufficient front readings: {len(sectors.front)} < {self.min_sector_readings}"
            )
            return False
        return True
    
    def _smooth_velocity(self, target_linear, target_angular):
        """
        Apply velocity ramping to prevent jerky motion.
        Respects acceleration limits.
        """
        max_delta_linear = self.velocity_ramp_rate / 10.0  # ~10Hz callback
        max_delta_angular = self.velocity_ramp_rate / 10.0
        
        # Clamp velocity changes
        linear_delta = max(
            min(target_linear - self.last_linear_vel, max_delta_linear),
            -max_delta_linear
        )
        angular_delta = max(
            min(target_angular - self.last_angular_vel, max_delta_angular),
            -max_delta_angular
        )
        
        smoothed_linear = self.last_linear_vel + linear_delta
        smoothed_angular = self.last_angular_vel + angular_delta
        
        self.last_linear_vel = smoothed_linear
        self.last_angular_vel = smoothed_angular
        
        return smoothed_linear, smoothed_angular
    
    def _decide_action(self, sectors):
        """
        Determine next action based on state machine and sensor data.
        Returns (linear_vel, angular_vel).
        """
        current_time = time.time()
        time_in_state = current_time - self.last_state_change
        
        front_min = sectors.get_safe_distance(sectors.front)
        front_avg = sectors.get_average_distance(sectors.front)
        
        # ------ STATE MACHINE ------
        
        if self.current_state == RobotState.MOVING_FORWARD:
            if front_min < self.critical_distance:
                # Imminent collision - back up immediately
                self._transition_state(RobotState.BACKING_UP)
                return self.backup_speed, 0.0
            elif front_avg < self.safe_distance:
                # Obstacle detected - start turning
                self._transition_state(RobotState.OBSTACLE_DETECTED)
            else:
                # Clear path - keep moving
                return self.forward_speed, 0.0
        
        if self.current_state == RobotState.OBSTACLE_DETECTED:
            # Decide turn direction based on side distances
            if time_in_state < self.state_transition_damping:
                return 0.0, 0.0  # Stop briefly to sense
            
            left_avg = sectors.get_average_distance(sectors.left)
            right_avg = sectors.get_average_distance(sectors.right)
            
            # Choose direction with more space
            turn_left = left_avg > right_avg
            
            self._transition_state(RobotState.TURNING)
            angular = self.turn_speed if turn_left else -self.turn_speed
            return 0.0, angular
        
        if self.current_state == RobotState.TURNING:
            # Check if we've turned enough to continue
            if front_avg > self.safe_distance + 0.2:  # Hysteresis
                self._transition_state(RobotState.MOVING_FORWARD)
                return self.forward_speed, 0.0
            
            # Continue turning in last direction
            angular = self.turn_speed if self.last_angular_vel > 0 else -self.turn_speed
            return 0.0, angular
        
        if self.current_state == RobotState.BACKING_UP:
            # Back up until we have clearance
            if front_min > self.safe_distance:
                self._transition_state(RobotState.MOVING_FORWARD)
                return self.forward_speed, 0.0
            # If stuck after backing, try turning
            if time_in_state > 2.0:
                self._transition_state(RobotState.STUCK)
            return self.backup_speed, 0.0
        
        if self.current_state == RobotState.STUCK:
            # Recovery behavior - turn aggressively
            return 0.0, self.turn_speed * 1.5
        
        return 0.0, 0.0
    
    def _transition_state(self, new_state):
        """Transition to new state with logging."""
        if new_state != self.current_state:
            self.get_logger().debug(
                f"State transition: {self.current_state.name} -> {new_state.name}"
            )
            self.current_state = new_state
            self.last_state_change = time.time()
    
    def scan_callback(self, msg):
        """Main sensor callback - orchestrates the decision loop."""
        try:
            # Segment and validate scan data
            sectors = self._segment_scan(msg)
            
            if not self._check_state_validity(sectors):
                self.get_logger().warn("Invalid scan data - stopping")
                self._publish_velocity(0.0, 0.0)
                return
            
            # Decide action based on state machine
            target_linear, target_angular = self._decide_action(sectors)
            
            # Apply velocity smoothing
            linear_vel, angular_vel = self._smooth_velocity(target_linear, target_angular)
            
            # Log current status
            self.get_logger().debug(
                f"State: {self.current_state.name} | "
                f"Front: {sectors.get_average_distance(sectors.front):.2f}m | "
                f"Cmd: linear={linear_vel:.2f}, angular={angular_vel:.2f}"
            )
            
            # Publish command
            self._publish_velocity(linear_vel, angular_vel)
            
        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {e}")
            self._publish_velocity(0.0, 0.0)
    
    def _publish_velocity(self, linear_x, angular_z):
        """Publish velocity command safely."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()