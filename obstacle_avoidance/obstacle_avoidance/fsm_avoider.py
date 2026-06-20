"""
Example: Finite State Machine (FSM) Obstacle Avoidance

This demonstrates the FSM approach where:
- Each state is explicit
- Transitions are edge-driven (sensor conditions)
- Entry/Exit actions can be defined per state
- State diagram is clear and visualizable

This pattern is excellent for complex state-dependent behaviors:
- Docking
- Multi-room navigation
- Battery charging when low
- Error recovery states
"""

import math
import time
from enum import Enum
from dataclasses import dataclass
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class NavigationState(Enum):
    """Define all possible states."""
    IDLE = 0
    MOVING_FORWARD = 1
    OBSTACLE_DETECTED = 2
    TURNING_LEFT = 3
    TURNING_RIGHT = 4
    BACKING_UP = 5
    STUCK_RECOVERY = 6


@dataclass
class SensorData:
    """Processed sensor information."""
    front_min: float
    front_avg: float
    left_avg: float
    right_avg: float
    is_valid: bool


@dataclass
class StateAction:
    """Command to publish from current state."""
    linear_vel: float
    angular_vel: float
    next_state: NavigationState = None  # If None, stay in current state


class State:
    """Base class for FSM states."""
    
    def __init__(self, name: str):
        self.name = name
        self.entry_time = None
        self.last_action = StateAction(0.0, 0.0)
    
    def on_enter(self):
        """Called when entering this state."""
        self.entry_time = time.time()
    
    def on_exit(self):
        """Called when leaving this state."""
        pass
    
    def time_in_state(self) -> float:
        """How long we've been in this state."""
        if self.entry_time is None:
            return 0.0
        return time.time() - self.entry_time
    
    def execute(self, sensor_data: SensorData) -> StateAction:
        """Execute state logic and return action + next state."""
        raise NotImplementedError


class IdleState(State):
    """Robot is stationary, waiting."""
    
    def __init__(self):
        super().__init__("IDLE")
    
    def execute(self, sensor_data: SensorData) -> StateAction:
        # From IDLE, we can start moving if no obstacle ahead
        if sensor_data.is_valid and sensor_data.front_avg > 1.0:
            return StateAction(0.0, 0.0, NavigationState.MOVING_FORWARD)
        return StateAction(0.0, 0.0)


class MovingForwardState(State):
    """Normal forward movement."""
    
    def __init__(self, forward_speed=0.3):
        super().__init__("MOVING_FORWARD")
        self.forward_speed = forward_speed
    
    def execute(self, sensor_data: SensorData) -> StateAction:
        if not sensor_data.is_valid:
            return StateAction(0.0, 0.0, NavigationState.IDLE)
        
        # If obstacle imminent, back up
        if sensor_data.front_min < 0.3:
            return StateAction(0.0, 0.0, NavigationState.BACKING_UP)
        
        # If obstacle ahead, detect
        if sensor_data.front_avg < 1.0:
            return StateAction(0.0, 0.0, NavigationState.OBSTACLE_DETECTED)
        
        # Clear path - keep moving
        return StateAction(self.forward_speed, 0.0)


class ObstacleDetectedState(State):
    """Obstacle found - analyze and decide turn direction."""
    
    def __init__(self):
        super().__init__("OBSTACLE_DETECTED")
    
    def execute(self, sensor_data: SensorData) -> StateAction:
        # Wait briefly to stabilize sensor readings (0.3s)
        if self.time_in_state() < 0.3:
            return StateAction(0.0, 0.0)  # Stop and sense
        
        # Decide which way to turn based on side clearance
        if sensor_data.left_avg > sensor_data.right_avg:
            return StateAction(0.0, 0.0, NavigationState.TURNING_LEFT)
        else:
            return StateAction(0.0, 0.0, NavigationState.TURNING_RIGHT)


class TurningLeftState(State):
    """Turn left until path is clear."""
    
    def __init__(self, turn_speed=1.0):
        super().__init__("TURNING_LEFT")
        self.turn_speed = turn_speed
    
    def execute(self, sensor_data: SensorData) -> StateAction:
        # Check if we've cleared the obstacle (with hysteresis)
        if sensor_data.front_avg > 1.2 and self.time_in_state() > 0.5:
            return StateAction(0.0, 0.0, NavigationState.MOVING_FORWARD)
        
        # Continue turning left
        return StateAction(0.0, self.turn_speed)


class TurningRightState(State):
    """Turn right until path is clear."""
    
    def __init__(self, turn_speed=1.0):
        super().__init__("TURNING_RIGHT")
        self.turn_speed = turn_speed
    
    def execute(self, sensor_data: SensorData) -> StateAction:
        # Check if we've cleared the obstacle (with hysteresis)
        if sensor_data.front_avg > 1.2 and self.time_in_state() > 0.5:
            return StateAction(0.0, 0.0, NavigationState.MOVING_FORWARD)
        
        # Continue turning right
        return StateAction(0.0, -self.turn_speed)


class BackingUpState(State):
    """Reverse to escape imminent collision."""
    
    def __init__(self, backup_speed=-0.2):
        super().__init__("BACKING_UP")
        self.backup_speed = backup_speed
    
    def execute(self, sensor_data: SensorData) -> StateAction:
        # Back up until we have clearance
        if sensor_data.front_avg > 1.0:
            return StateAction(0.0, 0.0, NavigationState.MOVING_FORWARD)
        
        # If stuck for too long, go to recovery
        if self.time_in_state() > 3.0:
            return StateAction(0.0, 0.0, NavigationState.STUCK_RECOVERY)
        
        # Continue backing
        return StateAction(self.backup_speed, 0.0)


class StuckRecoveryState(State):
    """Stuck - spin in place to find escape."""
    
    def __init__(self, turn_speed=2.0):
        super().__init__("STUCK_RECOVERY")
        self.turn_speed = turn_speed
    
    def execute(self, sensor_data: SensorData) -> StateAction:
        # Keep spinning until front is clear
        if sensor_data.front_avg > 1.0:
            return StateAction(0.0, 0.0, NavigationState.MOVING_FORWARD)
        
        # After 5 seconds of recovery, give up and stop
        if self.time_in_state() > 5.0:
            return StateAction(0.0, 0.0, NavigationState.IDLE)
        
        # Spin hard
        return StateAction(0.0, self.turn_speed * 1.5)


class FiniteStateMachineAvoider(Node):
    """
    FSM-based obstacle avoidance.
    
    ADVANTAGES:
    - Clear state diagram (visualizable)
    - Explicit state transitions
    - Easy to add complex state-dependent logic
    - Good for debugging (state is explicit)
    - Easy to add entry/exit actions per state
    
    GOOD FOR:
    - Complex multi-mode robots
    - Docking stations
    - Battery-aware navigation
    - Complex recovery behaviors
    
    CHALLENGES:
    - More code than reactive approaches
    - State explosion with too many states
    - Need to carefully define transitions
    """
    
    def __init__(self):
        super().__init__('fsm_avoider')
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create all states
        self.states = {
            NavigationState.IDLE: IdleState(),
            NavigationState.MOVING_FORWARD: MovingForwardState(forward_speed=0.3),
            NavigationState.OBSTACLE_DETECTED: ObstacleDetectedState(),
            NavigationState.TURNING_LEFT: TurningLeftState(turn_speed=1.0),
            NavigationState.TURNING_RIGHT: TurningRightState(turn_speed=1.0),
            NavigationState.BACKING_UP: BackingUpState(backup_speed=-0.2),
            NavigationState.STUCK_RECOVERY: StuckRecoveryState(turn_speed=2.0),
        }
        
        self.current_state = NavigationState.IDLE
        self.states[self.current_state].on_enter()
        
        self.get_logger().info("FSM Avoider initialized")
        self._log_state_info()
    
    def _transition_to_state(self, new_state: NavigationState):
        """Handle state transition with entry/exit."""
        if new_state == self.current_state:
            return
        
        # Exit old state
        self.states[self.current_state].on_exit()
        self.get_logger().info(
            f"Transition: {self.current_state.name} → {new_state.name}"
        )
        
        # Enter new state
        self.current_state = new_state
        self.states[self.current_state].on_enter()
    
    def _process_scan(self, msg) -> SensorData:
        """Extract and clean scan data."""
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
        """Main loop - execute current state and check transitions."""
        sensor_data = self._process_scan(msg)
        
        if not sensor_data.is_valid:
            self.get_logger().warn("Invalid scan data")
            self._publish_velocity(0.0, 0.0)
            return
        
        try:
            # Execute current state
            action = self.states[self.current_state].execute(sensor_data)
            
            # Check for state transition
            if action.next_state is not None:
                self._transition_to_state(action.next_state)
            
            # Log current state info
            self.get_logger().debug(
                f"State: {self.current_state.name} ({self.states[self.current_state].time_in_state():.2f}s) | "
                f"Front: {sensor_data.front_avg:.2f}m | "
                f"Cmd: ({action.linear_vel:.2f}, {action.angular_vel:.2f})"
            )
            
            # Publish velocity command
            self._publish_velocity(action.linear_vel, action.angular_vel)
            
        except Exception as e:
            self.get_logger().error(f"Error in FSM: {e}")
            self._publish_velocity(0.0, 0.0)
    
    def _publish_velocity(self, linear_x, angular_z):
        """Publish velocity command."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)
    
    def _log_state_info(self):
        """Log information about all states."""
        self.get_logger().info("FSM States:")
        for state in self.states.values():
            self.get_logger().info(f"  - {state.name}")


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = FiniteStateMachineAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
