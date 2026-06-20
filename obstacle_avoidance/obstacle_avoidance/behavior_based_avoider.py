"""
Example: Behavior-Based Obstacle Avoidance (Subsumption Architecture)

This demonstrates the Behavior-Based approach where multiple behaviors
compete, with higher priority behaviors suppressing lower ones.

This pattern is excellent for systems with multiple goals:
- Navigation
- Obstacle Avoidance  
- Target Following
- Charging
"""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


@dataclass
class SensorData:
    """Container for processed sensor information."""
    front_min: float
    front_avg: float
    left_avg: float
    right_avg: float
    is_valid: bool


class Behavior(ABC):
    """Base class for all behaviors."""
    
    @abstractmethod
    def can_run(self, sensor_data: SensorData) -> bool:
        """Check if this behavior should run given sensor data."""
        pass
    
    @abstractmethod
    def execute(self) -> tuple:
        """Execute behavior and return (linear_vel, angular_vel)."""
        pass
    
    @abstractmethod
    def priority(self) -> int:
        """Higher number = higher priority. 0-100."""
        pass


class EmergencyStop(Behavior):
    """
    HIGHEST PRIORITY - Stop immediately if collision is imminent.
    This behavior cannot be overridden.
    """
    
    def __init__(self, critical_distance=0.3):
        self.critical_distance = critical_distance
    
    def priority(self) -> int:
        return 100  # Highest priority
    
    def can_run(self, sensor_data: SensorData) -> bool:
        return sensor_data.front_min < self.critical_distance
    
    def execute(self) -> tuple:
        return 0.0, 0.0  # Complete stop


class BackUp(Behavior):
    """Back up if we're stuck (obstacle too close)."""
    
    def __init__(self, stuck_distance=0.5, backup_speed=-0.2):
        self.stuck_distance = stuck_distance
        self.backup_speed = backup_speed
    
    def priority(self) -> int:
        return 80
    
    def can_run(self, sensor_data: SensorData) -> bool:
        return sensor_data.front_avg < self.stuck_distance
    
    def execute(self) -> tuple:
        return self.backup_speed, 0.0


class AvoidObstacle(Behavior):
    """Turn away from obstacles when they're too close."""
    
    def __init__(self, safe_distance=1.0, turn_speed=1.0):
        self.safe_distance = safe_distance
        self.turn_speed = turn_speed
    
    def priority(self) -> int:
        return 50
    
    def can_run(self, sensor_data: SensorData) -> bool:
        return sensor_data.front_avg < self.safe_distance
    
    def execute(self, sensor_data: SensorData) -> tuple:
        # Turn toward the side with more space
        if sensor_data.left_avg > sensor_data.right_avg:
            return 0.0, self.turn_speed
        else:
            return 0.0, -self.turn_speed


class MoveForward(Behavior):
    """DEFAULT - Always available as fallback behavior."""
    
    def __init__(self, forward_speed=0.3):
        self.forward_speed = forward_speed
    
    def priority(self) -> int:
        return 10  # Lowest priority
    
    def can_run(self, sensor_data: SensorData) -> bool:
        return True  # Always available
    
    def execute(self) -> tuple:
        return self.forward_speed, 0.0


class BehaviorBasedAvoider(Node):
    """
    Behavior-based obstacle avoidance using subsumption architecture.
    
    ADVANTAGES:
    - Add/remove behaviors easily
    - Clear priority system
    - Decoupled behaviors
    - Intuitive to understand
    
    CHALLENGES:
    - Behaviors must not conflict
    - Priority order is critical
    """
    
    def __init__(self):
        super().__init__('behavior_based_avoider')
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize behaviors
        self.behaviors = [
            EmergencyStop(critical_distance=0.3),
            BackUp(stuck_distance=0.5),
            AvoidObstacle(safe_distance=1.0),
            MoveForward(forward_speed=0.3),
        ]
        
        # Sort by priority (highest first)
        self.behaviors.sort(key=lambda b: b.priority(), reverse=True)
        
        self.get_logger().info(f"Loaded {len(self.behaviors)} behaviors")
        for b in self.behaviors:
            self.get_logger().info(f"  - {b.__class__.__name__} (priority={b.priority()})")
    
    def _process_scan(self, msg) -> SensorData:
        """Clean and segment laser scan data."""
        front = []
        left = []
        right = []
        
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < 0.3:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            
            if -0.35 < angle < 0.35:
                front.append(r)
            elif 0.35 <= angle < 2.2:
                left.append(r)
            elif -2.2 < angle <= -0.35:
                right.append(r)
        
        if not front:
            return SensorData(
                front_min=float('inf'),
                front_avg=float('inf'),
                left_avg=float('inf'),
                right_avg=float('inf'),
                is_valid=False
            )
        
        return SensorData(
            front_min=min(front),
            front_avg=sum(front) / len(front),
            left_avg=sum(left) / len(left) if left else float('inf'),
            right_avg=sum(right) / len(right) if right else float('inf'),
            is_valid=True
        )
    
    def scan_callback(self, msg):
        """Main callback - arbitrate between behaviors."""
        sensor_data = self._process_scan(msg)
        
        if not sensor_data.is_valid:
            self.get_logger().warn("Invalid scan data")
            self._publish_velocity(0.0, 0.0)
            return
        
        # Find first (highest priority) behavior that can run
        active_behavior = None
        for behavior in self.behaviors:
            if behavior.can_run(sensor_data):
                active_behavior = behavior
                break
        
        if active_behavior is None:
            self.get_logger().error("No valid behavior found!")
            self._publish_velocity(0.0, 0.0)
            return
        
        # Execute the highest priority available behavior
        linear_vel, angular_vel = active_behavior.execute()
        
        self.get_logger().debug(
            f"Active: {active_behavior.__class__.__name__} | "
            f"Front: {sensor_data.front_avg:.2f}m | "
            f"Cmd: ({linear_vel:.2f}, {angular_vel:.2f})"
        )
        
        self._publish_velocity(linear_vel, angular_vel)
    
    def _publish_velocity(self, linear_x, angular_z):
        """Publish velocity command."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = BehaviorBasedAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
