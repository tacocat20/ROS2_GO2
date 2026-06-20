# Obstacle Avoidance: Software Architecture Patterns

This document explains different architectural approaches to obstacle avoidance and when to use each.

## 1. **MONOLITHIC REACTIVE** (Your Original Code)
```
Sensor Data → Process & Decision → Actuator Command (all in one callback)
```

### Characteristics:
- **Single callback** handles everything
- **No state** between decisions
- **Low latency** (direct sensor → action)
- **Highly coupled** code

### Pros:
- Simplest to understand
- Minimal overhead
- Fast response times

### Cons:
- **Oscillatory behavior** around thresholds (jitter)
- **Monolithic** - hard to test individual components
- **Magic numbers** scattered throughout
- **No recovery** behaviors
- **Difficult to extend** (adding new logic is messy)

### Example Behavior:
```
Distance: 0.99m → TURN
Distance: 1.01m → MOVE FORWARD
Distance: 0.99m → TURN
Distance: 1.01m → MOVE FORWARD  ← Oscillates around threshold!
```

---

## 2. **LAYERED/MODULAR** (Your Improved Code - avoider.py)
```
┌─ Sensor Layer ─────────────────┐
│  Clean & Segment Data          │
└──────────┬──────────────────────┘
           ↓
┌─ Decision Layer ───────────────┐
│  State Machine & Logic         │
└──────────┬──────────────────────┘
           ↓
┌─ Actuation Layer ──────────────┐
│  Smooth & Publish Commands     │
└────────────────────────────────┘
```

### Characteristics:
- **Separated concerns** - each layer has one job
- **State machine** prevents oscillation
- **Hysteresis** on thresholds
- **Velocity smoothing** for natural motion

### Pros:
- **Testable** - can test each layer independently
- **Configurable** - parameters via ROS2 declare_parameter
- **Smooth motion** - velocity ramping
- **Stable** - hysteresis prevents jitter
- **Maintainable** - clear structure

### Cons:
- **Slightly higher latency** (multiple function calls)
- **More code** (but clearer intent)
- **State complexity** (5 states to manage)

### State Transitions:
```
MOVING_FORWARD
    ↓ (obstacle detected, distance < safe_distance)
OBSTACLE_DETECTED (pause & sense)
    ↓ (decide turn direction)
TURNING (turn until clear)
    ↓ (front clear)
MOVING_FORWARD
```

---

## 3. **BEHAVIOR-BASED** (Subsumption Architecture)
```
Priority 0: EMERGENCY STOP (collision imminent)
    ↓ override if condition met
Priority 1: BACK UP (stuck)
    ↓ override if condition met
Priority 2: OBSTACLE AVOIDANCE (turn away)
    ↓ override if condition met
Priority 3: MOVE FORWARD (default)
```

### Characteristics:
- **Hierarchical behaviors** with override capability
- **Each behavior is independent**
- **Highest priority** behavior runs
- **Loosely coupled** - behaviors don't know about each other

### Example Implementation:
```python
class EmergencyStop(Behavior):
    def can_run(self, sensor_data):
        return sensor_data.front_min < 0.3
    
    def execute(self):
        return 0.0, 0.0  # Stop immediately

class Turning(Behavior):
    def can_run(self, sensor_data):
        return sensor_data.front_avg < safe_distance
    
    def execute(self):
        return 0.0, turn_speed

class Forward(Behavior):
    def can_run(self, sensor_data):
        return True  # Always available as fallback
    
    def execute(self):
        return forward_speed, 0.0
```

### Pros:
- **Modular** - add/remove behaviors easily
- **Clear priority** - no confusion about what runs
- **Scalable** - can have many behaviors
- **Intuitive** - mirrors human decision-making

### Cons:
- **Complexity in arbitration** - need good priority system
- **Potential conflicts** - behaviors may want different things
- **Harder to debug** - multiple behaviors active simultaneously

---

## 4. **FINITE STATE MACHINE (FSM) - Extended**
```
States: IDLE, MOVING, TURNING, BACKING, STUCK, CHARGING
Transitions: Edge-driven (sensor thresholds)
Guards: Hysteresis values
Actions: Per-state behavior
```

### Characteristics:
- **Explicit states** and transitions
- **Transition conditions** (guards)
- **State-specific actions** with entry/exit
- **Clear semantics** - state is the "mode" of operation

### Advantages:
- **Most robust** for complex behaviors
- **Easy to visualize** (draw state diagram)
- **Predictable** - state is explicit
- **Recovery handling** - can handle stuck situations

### Example State Diagram:
```
     ┌─────────────────────────────┐
     ↓                             │
  [MOVING_FORWARD]              (stuck > 2s)
  front > 1.2m
     ↓ (front < 1.0m)             │
  [OBSTACLE_DETECTED]        [STUCK]
  sense for 0.3s               │
     ↓ (decided direction)       └─ spin hard
  [TURNING]
  front < 1.2m
     ↓ (front > 1.4m)
  [MOVING_FORWARD]
```

---

## 5. **REACTIVE CONTROLLERS** (No State)
```
V_forward = max(0, safe_distance - current_distance) * Kp
V_turn = (right_distance - left_distance) * Kd
```

### Characteristics:
- **Pure physics-based** - P/PID control laws
- **Continuous functions** - no discrete states
- **Proportional response** - smoother than bang-bang

### Pros:
- **Mathematically elegant**
- **No state management**
- **Smooth transitions** (proportional response)

### Cons:
- **Requires tuning** - PID gains are tricky
- **Can oscillate badly** if gains are wrong
- **Sensitive to sensor noise**

---

## 6. **PLANNER + TRACKER** (Planning Approach)
```
Path Planner (global) → Local Path Planning (costmap) → Velocity Controller
```

### Characteristics:
- **Global path** to goal
- **Local costmap** built from sensors
- **Plan adjusted** as robot moves
- **Controller tracks** planned path

### Tools: Nav2, Move_Base, SLAM

### Pros:
- **Optimal path** over time
- **Can handle complex environments**
- **Goal-aware** - knows where it wants to go

### Cons:
- **Heavy computation** - needs global map
- **Slower response** - planning takes time
- **Complex to implement** - many components
- **Overkill for simple navigation**

---

## Comparison Table

| Pattern | State | Tuning | Complexity | Robustness | Use Case |
|---------|-------|--------|------------|-----------|----------|
| Monolithic Reactive | None | Low | Low | Low | Demo/simple |
| Layered (Your Code) | Yes | Medium | Medium | High | Production robots |
| Behavior-Based | Yes | Medium | Medium | Medium | Multi-goal systems |
| FSM | Yes | Medium | Medium | High | Complex robots |
| Reactive Controllers | None | High | Medium | Medium | Smooth motion needed |
| Planner + Tracker | Yes | High | High | Highest | Goal-driven navigation |

---

## Recommendations

### Choose **Layered** (Your Current Improved Code) if:
- ✓ Simple navigation without complex goals
- ✓ Need smooth, stable motion
- ✓ Want maintainable code
- ✓ Operating in partially known environments
- ✓ Limited computational resources

### Choose **Behavior-Based** if:
- ✓ Multiple behaviors needed (avoid + recharge + follow)
- ✓ Easy behavior swapping required
- ✓ Clear priority between behaviors

### Choose **FSM** if:
- ✓ Complex state-dependent logic
- ✓ Clear discrete modes (charging, exploring, avoiding)
- ✓ Need explicit recovery behaviors

### Choose **Planner + Tracker** if:
- ✓ Navigation to goal positions
- ✓ Complex environments with known maps
- ✓ Need optimal paths
- ✓ Sufficient CPU for planning

