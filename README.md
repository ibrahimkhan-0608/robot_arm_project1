#  6-Axis Robotic Arm: Kinematics & Path Planning

**Project 1 — Robotics & Automation Internship (DecodeLabs, Batch 2026)**

A complete ROS 2 robotics system: a simulated 6-axis industrial arm that
computes Inverse Kinematics, executes smooth collision-free spline
trajectories, and autonomously routes around obstacles — from Point A
to Point B with full safety verification.

---

##  Task Requirements — Delivered

| Requirement                                                                | Implementation                                                                                                      | Status |
| -------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- | ------ |
| Apply IK mathematics to calculate joint angles for a target XYZ coordinate | Closed-form 2-link geometric solver (law of cosines + atan2), verified to machine precision (0.00000 m error)       | ✅      |
| Utilize ROS and RViz for physics simulation                                | ROS 2 Humble, 6-node live pipeline, RViz 3D visualization                                                           | ✅      |
| Generate a smooth, collision-free trajectory spline                        | Cubic smoothstep spline (3τ²−2τ³, zero boundary velocities) + AABB collision checking + autonomous waypoint routing | ✅      |

---

##  System Architecture

```text
                        ┌──────────────────┐
   target position ────►│   planner_node   │──── joint goals ────┐
   (PointStamped)       │  the BRAIN:      │                     ▼
                        │  IK solve →      │            ┌────────────────┐
   pose feedback ──────►│  collision check │            │  driver_node   │
   (JointState)         │  → route via     │            │  the MUSCLES:  │
                        │  safe waypoints  │            │  smoothstep   │
                        └──────────────────┘            │  spline motion │
                                                        └───────┬────────┘
   arm poses (20 Hz) ◄──────────────────────────────────────────┘
        │
        ├──► robot_state_publisher ──► RViz (3D display)
        ├──► trail_node ──► golden trajectory spline (visualized)
        └──► obstacle_node ──► red pillar (visualized)
```

**The planning logic:**

```text
target → IK solve → check direct path
                       │
                    clear ──► execute directly
                       │
                  blocked ──► search candidate waypoints
                              (over the obstacle) → execute
                              leg 1 → verify arrival → leg 2
                       │
              nothing safe ──► REJECT (graceful failure)
```

##  The Mathematics

**Forward Kinematics** (angles → position) — verified by hand calculation:

```text
x = L1·sin(θ2) + L2·sin(θ2+θ3)
z = h  + L1·cos(θ2) + L2·cos(θ2+θ3)
```

**Inverse Kinematics** (position → angles) — the core solver:

```text
r  = √(dx² + dz²)                              reachability check
β  = arccos[(L1²+L2²−r²)/(2·L1·L2)]            law of cosines
θ3 = π − β                                     elbow angle
θ2 = atan2(dx,dz) − atan2(L2·sin θ3, L1+L2·cos θ3)   shoulder angle
```

**Trajectory spline** — cubic smoothstep with zero boundary velocities:

```text
q(τ) = q0 + (q1−q0)·(3τ² − 2τ³)     τ = t/T
```

**Collision checking** — AABB (axis-aligned bounding box) with safety
inflation (3 cm), 4 arm key points sampled along the spline (shoulder,
elbow, wrist, tool tip) at 40 samples per trajectory.

##  Engineering Highlights

* **Verified math, not assumed math** — every solver checked against
  hand calculations before wiring into the live robot
* **Closed-loop planning** — the planner watches pose feedback and
  verifies waypoint arrival before commanding the next leg
* **Graceful failure everywhere** — unreachable targets, blocked
  routes, and collision risks are rejected with clean errors, never
  crashes
* **Self-verifying IK** — every solution round-tripped through FK
  before execution (error < 1 mm reported live)
* **Time-scaled motion** — trajectory duration adapts to joint travel
  (max joint speed 1 rad/s, all joints synchronized)
* **Seamless retargeting** — new goals mid-motion chain smoothly from
  the live pose (no teleport, ever)

##  How to Run

**Environment:** ROS 2 Humble on Ubuntu 22.04 (WSL2), RViz display
via VcXsrv (Windows).

```bash
# 1. Start the X server (Windows side): XLaunch
#    Multiple windows → display 0 → start no client → disable access control

# 2. Launch the full system
source ~/ros2_ws/install/setup.bash
ros2 launch robot_arm autonomous.launch.py

# 3. Command a target (new terminal)
ros2 topic pub --once /target_position geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'base_link'}, point: {x: 0.15, y: 0.0, z: 0.65}}"

# The planner checks the path, routes around the obstacle if needed,
# and the arm glides to the target — trail visualized in RViz.
```

**Manual mode** (joint sliders, for testing):

```bash
ros2 launch robot_arm display.launch.py
```

##  Package Contents

| File                          | Role                                                       |
| ----------------------------- | ---------------------------------------------------------- |
| `urdf/six_axis_arm.urdf`      | 6-axis robot model (7 links, 6 revolute joints)            |
| `robot_arm/collision.py`      | Shared library: AABB math, arm key points, spline sampling |
| `robot_arm/planner_node.py`   | The brain: IK + collision-aware routing                    |
| `robot_arm/driver_node.py`    | Trajectory executor (smoothstep spline, 20 Hz)             |
| `robot_arm/ik_node.py`        | Standalone IK solver (verified building block)             |
| `robot_arm/fk_node.py`        | Forward kinematics node (verified building block)          |
| `robot_arm/trail_node.py`     | Tool-tip trajectory visualization                          |
| `robot_arm/obstacle_node.py`  | Obstacle rendering                                         |
| `launch/autonomous.launch.py` | Full autonomous system launch                              |
| `launch/display.launch.py`    | Manual slider mode launch                                  |

##  Tech Stack

ROS 2 Humble · Python 3 · RViz · URDF · Linux (Ubuntu 22.04 / WSL2)

##  Future Work

* 3D IK (base yaw for full XYZ targets)
* Self-collision checking (arm vs. own body)
* Multiple obstacles + graph search (A*/RRT-lite)
* Gazebo physics integration (joint dynamics, contact forces)
