"""
Collision module — AABB obstacle checking (Phase D).
Shared by: obstacle_node (visual), planner_node (trajectory checking).

Obstacle = Axis-Aligned Bounding Box (AABB):
    x ∈ [X_MIN, X_MAX], y ∈ [Y_MIN, Y_MAX], z ∈ [Z_MIN, Z_MAX]
A point collides iff ALL three range conditions hold — 6 comparisons,
one of the fastest tests in computing (why AABBs are industry standard).

INFLATION expands the checked volume by 3 cm beyond the real box:
conservative direction (links have thickness; samples can tunnel).
False alarms are acceptable — silent crashes are not.
"""
import math

# ---- The obstacle: pillar 6 cm x 20 cm x 65 cm, 22-28 cm in front of base ----
X_MIN, X_MAX = 0.22, 0.28
Y_MIN, Y_MAX = -0.10, 0.10
Z_MIN, Z_MAX = 0.0, 0.65

INFLATION = 0.03   # m — expand checked volume (safety margin)

# ---- Arm geometry (must match URDF / fk_node) ----
SHOULDER_HEIGHT = 0.20
UPPER_ARM = 0.30
FOREARM = 0.30
TOOL = 0.15


def check_point(x, y, z):
    """True if (x, y, z) is inside the inflated obstacle volume."""
    return (X_MIN - INFLATION <= x <= X_MAX + INFLATION and
            Y_MIN - INFLATION <= y <= Y_MAX + INFLATION and
            Z_MIN - INFLATION <= z <= Z_MAX + INFLATION)


def arm_key_points(theta1, theta2, theta3, theta5):
    """
    Arm skeleton sample points (partial FK, full 3D).
    Returns [(x, y, z)] for: shoulder, elbow, wrist, tool tip.
    Checking these 4 points approximates the whole arm body —
    the tool alone is not enough (a forearm could slice the box
    while the tip flies over it).
    """
    x_elbow = UPPER_ARM * math.sin(theta2)
    z_elbow = SHOULDER_HEIGHT + UPPER_ARM * math.cos(theta2)

    x_wrist = x_elbow + FOREARM * math.sin(theta2 + theta3)
    z_wrist = z_elbow + FOREARM * math.cos(theta2 + theta3)

    x_tool = x_wrist + TOOL * math.sin(theta2 + theta3 + theta5)
    z_tool = z_wrist + TOOL * math.cos(theta2 + theta3 + theta5)

    # Base rotation lifts planar points into 3D
    c, s = math.cos(theta1), math.sin(theta1)

    return [
        (0.0, 0.0, SHOULDER_HEIGHT),            # shoulder (column axis)
        (x_elbow * c, x_elbow * s, z_elbow),    # elbow
        (x_wrist * c, x_wrist * s, z_wrist),    # wrist
        (x_tool * c, x_tool * s, z_tool),       # tool tip
    ]


def arm_collides(theta1, theta2, theta3, theta5):
    """True if any arm key point is inside the obstacle."""
    return any(check_point(*p) for p in
               arm_key_points(theta1, theta2, theta3, theta5))


# ---- Trajectory sampling (used by the planner, next step) ----

def sample_spline(q_start, q_goal, n=40):
    """
    Sample n poses along the smoothstep spline between two joint vectors
    (same motion law as the driver: q = q0 + (q1-q0)*(3τ²-2τ³)).
    Returns list of joint vectors (6 elements each).
    """
    poses = []
    for i in range(n + 1):
        tau = i / n
        s = tau * tau * (3.0 - 2.0 * tau)
        q = [a + (b - a) * s for a, b in zip(q_start, q_goal)]
        poses.append(q)
    return poses


def spline_collides(q_start, q_goal, n=40):
    """
    True if ANY sampled pose along the spline collides.
    Dense sampling (40 points) prevents tunneling through the box.
    q vectors: [base_yaw, shoulder, elbow, wrist_roll, wrist_pitch, tool_roll]
    """
    for q in sample_spline(q_start, q_goal, n):
        if arm_collides(q[0], q[1], q[2], q[4]):
            return True
    return False