"""
Example: Reactive Controller (Proportional Control)

This demonstrates the Reactive Controller approach where behavior
emerges from continuous control laws (P/PI/PID).

This pattern:
- Converts sensor distances to velocities using math functions
- No discrete states
- Smooth transitions (proportional response)
- Mimics physics-based behavior

Pros:
- Mathematically elegant
- Smooth motion
- No state management needed

Cons:
- Requires tuning (Kp, Ki, Kd gains)
- Can oscillate with bad gains
- Less predictable than FSM
"""

import math
from dataclasses import dataclass
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


@dataclass
class SensorData:
    """Processed sensor information."""
    front_min: float
    front_avg: float
    left_avg: float
    right_avg: float
    left_min: float
    right_min: float
    is_valid: bool


class ProportionalController:
    """
    Proportional controller: output = Kp * error
    
    For obstacle avoidance:
    - Forward velocity is proportional to distance (push harder when far away)
    - Turn rate is proportional to left/right difference
    """
    
    def __init__(self, setpoint=1.5, kp=0.5):
        self.setpoint = setpoint  # Desired distance
        self.kp = kp  # Proportional gain
    
    def compute(self, measured_distance: float) -> float:
        """
        Compute control output.
        
        If distance > setpoint: positive output (move forward)
        If distance < setpoint: negative output (back up / turn away)
        """
        error = measured_distance - self.setpoint
        return self.kp * error


class ReactiveAvoider(Node):
    """
    Reactive obstacle avoidance using proportional control laws.
    
    Control Laws:
    - Forward velocity: Proportional to distance ahead
      v_forward = Kp_forward * (distance_ahead - safe_distance)
    
    - Turn rate: Proportional to left-right asymmetry
      v_turn = Kp_turn * (left_distance - right_distance)
    
    - Emergency stop: If too close, override with stop command
    
    ADVANTAGES:
    - Mathematically clean
    - Smooth motion (no bang-bang)
    - Low computational cost
    - Easy to visualize as curves
    
    DISADVANTAGES:
    - Requires careful gain tuning
    - Can oscillate around setpoint
    - No state management (less predictable)
    - May not handle complex scenarios well
    """
    
    def __init__(self):
        super().__init__('reactive_avoider')
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Declare control parameters
        self.declare_parameter('kp_forward', 0.3)  # Forward velocity gain
        self.declare_parameter('kp_turn', 0.8)     # Turn rate gain
        self.declare_parameter('safe_distance', 1.0)
        self.declare_parameter('critical_distance', 0.3)
        self.declare_parameter('max_forward_speed', 0.5)
        self.declare_parameter('max_turn_speed', 2.0)
        self.declare_parameter('min_sensor_range', 0.3)
        
        # Get parameters
        self.kp_forward = self.get_parameter('kp_forward').value
        self.kp_turn = self.get_parameter('kp_turn').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.max_forward_speed = self.get_parameter('max_forward_speed').value
        self.max_turn_speed = self.get_parameter('max_turn_speed').value
        self.min_sensor_range = self.get_parameter('min_sensor_range').value
        
        # Create controllers
        self.forward_controller = ProportionalController(
            setpoint=self.safe_distance,
            kp=self.kp_forward
        )
        self.turn_controller = ProportionalController(
            setpoint=0.0,  # Symmetric left-right
            kp=self.kp_turn
        )
        
        # Velocity smoothing (optional but recommended)
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.smoothing_alpha = 0.7  # Lower = more smoothing
        
        self.get_logger().info("Reactive Avoider initialized with proportional controllers")
        self.get_logger().info(f"  Kp_forward={self.kp_forward}, Kp_turn={self.kp_turn}")
    
    def _process_scan(self, msg) -> SensorData:
        """Extract and clean scan data."""
        front = []
        left = []
        right = []
        
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < self.min_sensor_range:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            
            # Sector angles
            if -0.35 < angle < 0.35:  # Front ±20°
                front.append(r)
            elif 0.35 <= angle < 2.2:  # Left
                left.append(r)
            elif -2.2 < angle <= -0.35:  # Right
                right.append(r)
        
        if not front:
            return SensorData(
                front_min=float('inf'),
                front_avg=float('inf'),
                left_avg=float('inf'),
                right_avg=float('inf'),
                left_min=float('inf'),
                right_min=float('inf'),
                is_valid=False
            )
        
        left_avg = sum(left) / len(left) if left else float('inf')
        right_avg = sum(right) / len(right) if right else float('inf')
        
        return SensorData(
            front_min=min(front),
            front_avg=sum(front) / len(front),
            left_avg=left_avg,
            right_avg=right_avg,
            left_min=min(left) if left else float('inf'),
            right_min=min(right) if right else float('inf'),
            is_valid=True
        )
    
    def _compute_velocities(self, sensor_data: SensorData) -> tuple:
        """
        Compute velocities using proportional control laws.
        
        Returns:
            (linear_velocity, angular_velocity)
        """
        # Safety override: emergency stop if too close
        if sensor_data.front_min < self.critical_distance:
            return 0.0, 0.0
        
        # 1. Forward velocity proportional to distance
        # Further away → faster forward
        # Closer → slower (or backward)
        raw_linear = self.forward_controller.compute(sensor_data.front_avg)
        
        # 2. Turn rate proportional to left-right asymmetry
        # More space on left → turn left
        # More space on right → turn right
        asymmetry = sensor_data.left_avg - sensor_data.right_avg
        raw_angular = self.turn_controller.compute(asymmetry)
        
        # 3. Saturation - limit to max speeds
        linear_vel = max(-0.3, min(raw_linear, self.max_forward_speed))
        angular_vel = max(-self.max_turn_speed, min(raw_angular, self.max_turn_speed))
        
        # 4. Optional: Low-pass filter for smoothing
        smoothed_linear = (
            self.smoothing_alpha * linear_vel +
            (1 - self.smoothing_alpha) * self.last_linear
        )
        smoothed_angular = (
            self.smoothing_alpha * angular_vel +
            (1 - self.smoothing_alpha) * self.last_angular
        )
        
        self.last_linear = smoothed_linear
        self.last_angular = smoothed_angular
        
        return smoothed_linear, smoothed_angular
    
    def scan_callback(self, msg):
        """Main callback - compute and publish velocity."""
        sensor_data = self._process_scan(msg)
        
        if not sensor_data.is_valid:
            self.get_logger().warn("Invalid scan data")
            self._publish_velocity(0.0, 0.0)
            return
        
        # Compute velocities from control laws
        linear_vel, angular_vel = self._compute_velocities(sensor_data)
        
        self.get_logger().debug(
            f"Front: {sensor_data.front_avg:.2f}m | "
            f"Asymmetry: {sensor_data.left_avg - sensor_data.right_avg:.2f}m | "
            f"Cmd: linear={linear_vel:.3f}, angular={angular_vel:.3f}"
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
    node = ReactiveAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
