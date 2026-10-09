import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


# ===== Robot geometry (numbers from arm.urdf) =====
SHOULDER_Z = 0.075            # joint2 height: joint1 at 0.05 + 0.025
L1 = 0.15                     # joint2 -> joint3 (upper arm)
GRASP_IN_PALM = 0.045         # grasp point, measured from the palm along the tool
L2 = 0.12 + GRASP_IN_PALM     # joint3 -> grasp point (forearm + palm offset)

# ===== Gripper =====
OPEN = 0.0
CLOSED = 0.005                # V-tips just touch a 2.6 cm cube here

# ===== Scene =====
CUBE = 0.026                  # cube edge length
POST_H = 0.06                 # pedestal height
PICK_R = 0.25                 # distance of the pedestals from the base
PICK_ANGLES = [-1.0, -0.6, -0.2]     # where the 3 cubes stand (rad)
CUBE_COLORS = [(0.95, 0.80, 0.10), (0.10, 0.75, 0.85), (0.60, 0.30, 0.85)]

BOX_R = 0.25                  # distance of the box from the base
BOX_ANGLE = 1.2               # direction of the box (rad)
BOX_IN = 0.08                 # inner width of the box
WALL = 0.004                  # wall thickness
BOX_H = 0.044                 # wall height
SLOTS = [(-0.02, -0.02), (-0.02, 0.02), (0.02, 0.0)]  # cube spots inside the box

LIFT = 0.05                   # how far to lift above a cube
DROP_Z = 0.12                 # grasp-point height when releasing over the box
HOME = [0.0, 0.2, 1.2]        # joint1, joint2, joint3 at rest
LIMIT = 1.57                  # joint limit from the URDF


def ik(x, y, z):
    """Joint angles that put the grasp point at (x, y, z)."""
    j1 = math.atan2(y, x)                         # turn the base to face the target
    r = math.hypot(x, y)                          # how far out
    dz = z - SHOULDER_Z                           # how high, seen from the shoulder
    c3 = (r * r + dz * dz - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    j3 = math.acos(max(-1.0, min(1.0, c3)))       # elbow bend (cosine rule)
    j2 = math.atan2(r, dz) - math.atan2(L2 * math.sin(j3), L1 + L2 * math.cos(j3))
    return [j1, j2, j3]


def polar(r, a, z):
    """Distance + direction -> x, y, z."""
    return (r * math.cos(a), r * math.sin(a), z)


def box_point(u, v):
    """A point inside the box (u = outward, v = sideways) -> world x, y."""
    bx, by, _ = polar(BOX_R, BOX_ANGLE, 0.0)
    c, s = math.cos(BOX_ANGLE), math.sin(BOX_ANGLE)
    return bx + u * c - v * s, by + u * s + v * c


def yaw_quat(a):
    """Rotation around Z as a quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def smooth(s):
    """0..1 -> 0..1 with a slow start and a slow stop, like a servo."""
    return 0.5 - 0.5 * math.cos(math.pi * s)


class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'pick_scene', 10)

        self.joints = list(HOME)
        self.finger = OPEN
        self.reset_cubes()
        self.steps = self.make_plan()
        self.step_i = 0
        self.step_t = 0.0
        self.step_start = None

        self.dt = 0.05                                  # 20 ticks per second
        self.timer = self.create_timer(self.dt, self.tick)

    # ---------- where the cubes are ----------
    def reset_cubes(self):
        self.cubes = []
        for a in PICK_ANGLES:
            pos = polar(PICK_R, a, POST_H + CUBE / 2)
            self.cubes.append({'state': 'world', 'pos': pos, 'yaw': a})

    # ---------- the job as a list of steps ----------
    def make_plan(self):
        steps = []
        for i in range(len(PICK_ANGLES)):
            cx, cy, cz = self.cubes[i]['pos']
            sx, sy = box_point(*SLOTS[i])
            steps += [
                ('move', ik(cx, cy, cz + LIFT), 1.5),   # above the cube
                ('move', ik(cx, cy, cz), 1.0),          # down to the cube
                ('grip', CLOSED, 0.6),                  # close the gripper
                ('attach', i, 0.0),                     # cube now rides with the palm
                ('move', ik(cx, cy, cz + LIFT), 1.0),   # lift it
                ('move', ik(sx, sy, DROP_Z), 2.0),      # carry it over its box spot
                ('grip', OPEN, 0.6),                    # open the gripper
                ('drop', i, 0.0),                       # cube falls into the box
            ]
        steps += [
            ('move', HOME, 2.0),                        # back to rest
            ('wait', None, 2.0),
            ('reset', None, 0.0),                       # cubes back on the posts, repeat
        ]

        for kind, value, _ in steps:                    # safety check against URDF limits
            if kind == 'move' and any(abs(j) > LIMIT for j in value):
                self.get_logger().warn(f'Pose outside joint limits: {value}')
        return steps

    # ---------- runs 20 times per second ----------
    def tick(self):
        kind, value, duration = self.steps[self.step_i]

        if self.step_start is None:                     # first tick of this step
            self.step_start = (list(self.joints), self.finger)
            self.step_t = 0.0
            if kind == 'attach':
                phi = self.joints[1] + self.joints[2]   # how much the palm is tilted
                self.cubes[value] = {'state': 'held', 'phi': phi}
            elif kind == 'drop':
                x, y = box_point(*SLOTS[value])
                self.cubes[value] = {'state': 'world',
                                     'pos': (x, y, WALL + CUBE / 2), 'yaw': BOX_ANGLE}
            elif kind == 'reset':
                self.reset_cubes()

        self.step_t += self.dt
        s = smooth(min(1.0, self.step_t / duration)) if duration > 0 else 1.0
        start_joints, start_finger = self.step_start

        if kind == 'move':
            self.joints = [a + (b - a) * s for a, b in zip(start_joints, value)]
        elif kind == 'grip':
            self.finger = start_finger + (value - start_finger) * s

        if self.step_t >= duration:                     # step done -> next one
            self.step_i = (self.step_i + 1) % len(self.steps)
            self.step_start = None

        self.publish_joints()
        self.publish_scene()

    # ---------- talk to robot_state_publisher ----------
    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # right_finger_joint is a mimic joint; sending its value too is harmless and safe
        msg.name = ['joint1', 'joint2', 'joint3', 'finger_joint', 'right_finger_joint']
        msg.position = self.joints + [self.finger, self.finger]
        self.joint_pub.publish(msg)

    # ---------- draw cubes, posts and box ----------
    def marker(self, ns, mid, shape, frame, pos, quat, size, color):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = Time().to_msg()                # zero time = "use the latest TF"
        m.ns, m.id = ns, mid
        m.type = shape
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = quat
        m.scale.x, m.scale.y, m.scale.z = size
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        m.frame_locked = True                           # follow the frame as it moves
        return m

    def publish_scene(self):
        arr = MarkerArray()

        # posts the cubes stand on
        for i, a in enumerate(PICK_ANGLES):
            arr.markers.append(self.marker(
                'posts', i, Marker.CYLINDER, 'base_link',
                polar(PICK_R, a, POST_H / 2), (0.0, 0.0, 0.0, 1.0),
                (0.016, 0.016, POST_H), (0.55, 0.55, 0.55)))

        # the box: bottom + 4 walls
        outer = BOX_IN + 2 * WALL
        half = BOX_IN / 2 + WALL / 2
        walls = [  # (u, v, z, size_u, size_v, size_z)
            (0.0, 0.0, WALL / 2, outer, outer, WALL),
            (half, 0.0, BOX_H / 2, WALL, outer, BOX_H),
            (-half, 0.0, BOX_H / 2, WALL, outer, BOX_H),
            (0.0, half, BOX_H / 2, outer, WALL, BOX_H),
            (0.0, -half, BOX_H / 2, outer, WALL, BOX_H),
        ]
        for i, (u, v, z, su, sv, sz) in enumerate(walls):
            x, y = box_point(u, v)
            arr.markers.append(self.marker(
                'box', i, Marker.CUBE, 'base_link', (x, y, z),
                yaw_quat(BOX_ANGLE), (su, sv, sz), (0.55, 0.35, 0.20)))

        # the cubes
        for i, cube in enumerate(self.cubes):
            if cube['state'] == 'held':
                # drawn in the palm frame, so TF carries it with the gripper
                q = (0.0, math.sin(-cube['phi'] / 2), 0.0, math.cos(-cube['phi'] / 2))
                arr.markers.append(self.marker(
                    'cubes', i, Marker.CUBE, 'palm', (0.0, 0.0, GRASP_IN_PALM),
                    q, (CUBE, CUBE, CUBE), CUBE_COLORS[i]))
            else:
                arr.markers.append(self.marker(
                    'cubes', i, Marker.CUBE, 'base_link', cube['pos'],
                    yaw_quat(cube['yaw']), (CUBE, CUBE, CUBE), CUBE_COLORS[i]))

        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = PickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()