# How to Extend Each Architecture

This guide shows how to add new capabilities to each pattern.

---

## Adding Wall Following to Each Pattern

Imagine you want the robot to follow a wall on its left side.

### 1. **Layered Approach** (avoider.py)

Add a new state:

```python
class RobotState(Enum):
    # ... existing states ...
    WALL_FOLLOWING = 6  # NEW

# In _decide_action():
if self.current_state == RobotState.WALL_FOLLOWING:
    wall_error = sectors.left_avg - DESIRED_WALL_DISTANCE
    angular = -wall_error * KP_WALL  # Steer to maintain distance
    linear = 0.2
    
    # Exit wall following if obstacle ahead
    if sectors.front_avg < self.safe_distance:
        self._transition_state(RobotState.OBSTACLE_DETECTED)
    return linear, angular
```

**Pros:** Simple, integrates with existing state machine
**Cons:** Need to think about interactions with other states

---

### 2. **Behavior-Based Approach** (behavior_based_avoider.py)

Add a new behavior:

```python
class WallFollowing(Behavior):
    """Follow a wall on the left side."""
    
    def __init__(self, desired_distance=0.5, kp=0.8):
        self.desired_distance = desired_distance
        self.kp = kp
    
    def priority(self) -> int:
        return 40  # Lower than obstacle avoidance (50)
    
    def can_run(self, sensor_data: SensorData) -> bool:
        # Only wall-follow if wall exists and is stable
        return (sensor_data.left_avg < 2.0 and 
                sensor_data.front_avg > 1.0)
    
    def execute(self, sensor_data: SensorData) -> tuple:
        # Maintain distance from wall
        error = sensor_data.left_avg - self.desired_distance
        angular = error * self.kp
        return 0.2, angular  # (forward_speed, turn_rate)

# Add to behaviors list:
self.behaviors = [
    EmergencyStop(...),
    BackUp(...),
    WallFollowing(...),        # NEW
    AvoidObstacle(...),
    MoveForward(...),
]
```

**Pros:** New behavior is completely isolated, automatically prioritized
**Cons:** Need to ensure priority order is correct

---

### 3. **FSM Approach** (fsm_avoider.py)

Add a new state class:

```python
class WallFollowingState(State):
    """Follow wall on left side."""
    
    def __init__(self, desired_wall_distance=0.5):
        super().__init__("WALL_FOLLOWING")
        self.desired_wall_distance = desired_wall_distance
        self.kp = 0.8
    
    def execute(self, sensor_data: SensorData) -> StateAction:
        # Maintain distance from wall
        error = sensor_data.left_avg - self.desired_wall_distance
        angular = error * self.kp
        
        # Check transitions
        if sensor_data.front_avg < 1.0:
            return StateAction(0.0, 0.0, NavigationState.OBSTACLE_DETECTED)
        
        if sensor_data.left_avg > 2.0:  # Lost wall
            return StateAction(0.0, 0.0, NavigationState.MOVING_FORWARD)
        
        # Continue wall following
        return StateAction(0.2, angular)

# Add state to FSM:
self.states[NavigationState.WALL_FOLLOWING] = WallFollowingState()

# Add transitions from MOVING_FORWARD:
if action.next_state is None and sensor_data.left_avg < 1.5:
    self._transition_to_state(NavigationState.WALL_FOLLOWING)
```

**Pros:** Clear state with explicit entry/exit points
**Cons:** Need to update state transitions

---

### 4. **Reactive Controller** (reactive_avoider.py)

Add to the control law:

```python
def _compute_velocities(self, sensor_data: SensorData) -> tuple:
    # ... existing forward control ...
    
    # NEW: Wall following term
    # Want to maintain left_distance = desired_distance
    wall_error = sensor_data.left_avg - 0.5  # 0.5m from wall
    wall_control = 0.5 * wall_error  # Proportional gain
    
    # Combine obstacle avoidance and wall following
    asymmetry = sensor_data.left_avg - sensor_data.right_avg
    avoidance_control = 0.8 * asymmetry
    
    # Total turn command (wall following overrides if stronger)
    raw_angular = avoidance_control + wall_control
    
    # ... rest of saturation/smoothing ...
    return linear_vel, angular_vel
```

**Pros:** Just add another term to the control law
**Cons:** Multiple control terms can interfere (need gain tuning)

---

## Adding Target Following

Now add the ability to follow a target detected by a camera or other sensor.

### 1. **Layered Approach**

```python
class RobotState(Enum):
    # ... existing ...
    FOLLOWING_TARGET = 7  # NEW

def __init__(self):
    # ... existing ...
    self.target_sub = self.create_subscription(
        Float32,  # Target angle from center
        '/target_angle',
        self.target_callback,
        10
    )
    self.target_angle = None  # Will be set by callback

def target_callback(self, msg):
    self.target_angle = msg.data

def _decide_action(self, sectors):
    # ... existing states ...
    
    if self.current_state == RobotState.FOLLOWING_TARGET:
        if self.target_angle is None:
            # Lost target
            self._transition_state(RobotState.MOVING_FORWARD)
            return self.forward_speed, 0.0
        
        if sectors.front_min < self.critical_distance:
            # Obstacle in way
            self._transition_state(RobotState.BACKING_UP)
            return self.backup_speed, 0.0
        
        # Turn toward target
        angular = self.target_angle * 0.5
        angular = max(-self.turn_speed, min(angular, self.turn_speed))
        return 0.2, angular
```

---

### 2. **Behavior-Based Approach**

```python
class FollowTarget(Behavior):
    """Follow a detected target (camera, etc)."""
    
    def __init__(self, max_tracking_age=1.0):
        self.target_angle = None
        self.last_target_time = None
        self.max_tracking_age = max_tracking_age
    
    def update_target(self, angle: float):
        """Called when target is detected."""
        self.target_angle = angle
        self.last_target_time = time.time()
    
    def priority(self) -> int:
        return 75  # High priority, but not above emergency
    
    def can_run(self, sensor_data: SensorData) -> bool:
        # Can track if we have a recent target and path is clear
        if self.last_target_time is None:
            return False
        age = time.time() - self.last_target_time
        return age < self.max_tracking_age and sensor_data.front_avg > 1.0
    
    def execute(self) -> tuple:
        # Turn toward target, move forward
        angular = self.target_angle * 0.5
        return 0.3, angular

# In main callback, add:
follow_behavior.update_target(incoming_target_angle)
```

**Advantage:** Target following is a completely isolated behavior with clear priority

---

### 3. **FSM Approach**

```python
class FollowingTargetState(State):
    """Following detected target."""
    
    def __init__(self):
        super().__init__("FOLLOWING_TARGET")
        self.target_angle = None
        self.last_update_time = None
    
    def update_target(self, angle: float):
        self.target_angle = angle
        self.last_update_time = time.time()
    
    def execute(self, sensor_data: SensorData) -> StateAction:
        # Check if target is still valid
        if self.target_angle is None or time.time() - self.last_update_time > 1.0:
            return StateAction(0.0, 0.0, NavigationState.MOVING_FORWARD)
        
        # Check for obstacles
        if sensor_data.front_avg < 1.0:
            return StateAction(0.0, 0.0, NavigationState.OBSTACLE_DETECTED)
        
        # Follow target
        angular = self.target_angle * 0.5
        return StateAction(0.3, angular)

# In main callback:
self.states[NavigationState.FOLLOWING_TARGET].update_target(incoming_angle)
```

---

### 4. **Reactive Controller**

```python
def __init__(self):
    # ... existing ...
    self.target_sub = self.create_subscription(...)
    self.target_angle = None
    self.target_controller = ProportionalController(
        setpoint=0.0,  # Center camera
        kp=0.5
    )

def target_callback(self, msg):
    self.target_angle = msg.data

def _compute_velocities(self, sensor_data: SensorData) -> tuple:
    # ... existing forward control ...
    
    # If we have a target, track it
    if self.target_angle is not None:
        # Target tracking overrides obstacle avoidance
        angular = self.target_controller.compute(self.target_angle)
    else:
        # Normal obstacle avoidance
        asymmetry = sensor_data.left_avg - sensor_data.right_avg
        angular = 0.8 * asymmetry
    
    # ... saturation ...
    return linear_vel, angular_vel
```

---

## Complexity Comparison

| Feature | Layered | Behavior | FSM | Reactive |
|---------|---------|----------|-----|----------|
| Add wall follow | ⭐⭐ | ⭐ | ⭐⭐ | ⭐⭐ |
| Add target follow | ⭐⭐ | ⭐ | ⭐⭐ | ⭐⭐ |
| Add battery check | ⭐⭐⭐ | ⭐ | ⭐⭐ | ⭐⭐⭐ |
| Add multi-room nav | ⭐⭐⭐ | ⭐⭐ | ⭐ | ⭐⭐⭐⭐ |

---

## Pattern Recommendations Based on Feature

### Simple Features (Obstacle Avoidance Only)
→ Use **Layered** or **Reactive**

### Features with Priorities (avoid > follow > navigate)
→ Use **Behavior-Based**

### Features with Discrete Modes (charging, docking, exploring)
→ Use **FSM**

### Features with Smooth Control (wall following, trajectory tracking)
→ Use **Reactive** or **Layered**

---

## Testing Your Extensions

### Debug Single Behavior
```python
# In behavior_based_avoider.py
def can_run(self, sensor_data: SensorData) -> bool:
    result = sensor_data.left_avg < 2.0
    print(f"WallFollowing.can_run: {result}")  # Debug
    return result
```

### Log State Transitions
```python
# In fsm_avoider.py
self.get_logger().info(
    f"Transition: {self.current_state.name} → {new_state.name}"
)
```

### Plot Sensor Data
```python
# Capture to CSV and plot
self.get_logger().info(
    f"{sensor_data.front_avg},{sensor_data.left_avg},{sensor_data.right_avg}"
)
```

---

## When to Refactor to Another Pattern

| Sign | Recommendation |
|------|-----------------|
| Adding 5th+ state | → Consider behavior-based |
| States are complex | → Use explicit FSM |
| Multiple control loops | → Use reactive controller |
| Behaviors compete | → Use behavior-based |
| Smooth motion needed | → Add velocity ramping or reactive |

