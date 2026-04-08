#!/usr/bin/env python3

import time
import math
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints,
                              PositionConstraint, OrientationConstraint,
                              JointConstraint, PlanningOptions)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from tf2_ros import Buffer, TransformListener
import tf_transformations

# ---- singularity helper import (safe) ----
try:
    from .poe_singularity import is_singular
except Exception:
    def is_singular(thetalist, Slist, threshold=1e-5):
        # Fallback: always “not singular”
        return False, 1.0

# Screw axes for RX-200
Slist = np.array([
    [0.0, 0.0, 0.0, 0.0, 1.0],
    [0.0, 1.0, 1.0, 1.0, 0.0],
    [1.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, -0.10457, -0.30457, -0.30457, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.30457],
    [0.0, 0.0, 0.05, 0.25, 0.0]
], dtype=float)

SINGULARITY_JOINT_ORDER = [
    "waist",
    "shoulder",
    "elbow",
    "wrist_angle",
    "wrist_rotate"
]


class MoveItEEClient(Node):
    def __init__(self, singularity_threshold: float = 1e-5):
        super().__init__('rx200_moveit_control')

        # MoveIt action client
        self._client = ActionClient(self, MoveGroup, '/move_action')

        self.get_logger().info("Waiting for MoveIt action server...")
        while not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('Waiting for MoveIt action server...')

        self.group_name_arm = 'interbotix_arm'
        self.group_name_gripper = 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.gripper_joint = 'left_finger'
        self.ref_frame = 'world'

        self.singularity_threshold = singularity_threshold

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_joint_states = None
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        # for vision
        self.pose_reached_publisher = self.create_publisher(
            Float32MultiArray, 'pose_reached', 10
        )

        # coords from GUI / vision
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'goal_coordinates',
            self.coordinate_receiver,
            10
        )

        self.goal_lock = threading.Lock()
        self.goal_in_progress = False
        self.current_goal_handle = None

        # initial gripper pose
        self.declare_parameter('start_state_gripper', value=True)
        try:
            self.send_gr_pose(self.get_parameter('start_state_gripper').value)
        except Exception as e:
            self.get_logger().warning(f"Failed to set initial gripper pose: {e}")

        self._print_startup_manipulability()
        self.verbose = False
        self.get_logger().info("MoveItEEClient initialized.")

    # ========== Joint / singularity helpers ==========

    def _joint_cb(self, msg: JointState):
        self.latest_joint_states = msg

    def get_joint_position(self, joint_name: str) -> float:
        if self.latest_joint_states is None:
            return 0.0
        try:
            idx = list(self.latest_joint_states.name).index(joint_name)
            return float(self.latest_joint_states.position[idx])
        except Exception:
            return 0.0

    def get_current_thetalist(self) -> np.ndarray:
        vals = [self.get_joint_position(j) for j in SINGULARITY_JOINT_ORDER]
        return np.array(vals, dtype=float)

    def _print_startup_manipulability(self):
        thetas = self.get_current_thetalist()
        try:
            is_sing, m = is_singular(thetas, Slist, threshold=self.singularity_threshold)
            if is_sing:
                self.get_logger().warn(f"Startup: near singularity, m={m:.8e}")
            else:
                self.get_logger().info(f"Startup manipulability m={m:.8e}")
        except Exception as e:
            self.get_logger().warn(f"Startup manipulability check failed: {e}")

    def _check_singularity_and_log(self):
        thetas = self.get_current_thetalist()
        try:
            is_sing, m = is_singular(thetas, Slist, threshold=self.singularity_threshold)
            if is_sing:
                self.get_logger().warn(f"Near singularity: m={m:.8e}")
            else:
                if self.verbose:
                    self.get_logger().info(f"m={m:.8e}")
            return is_sing, m
        except Exception as e:
            self.get_logger().warn(f"Singularity check failed: {e}")
            return False, 0.0

    # ========== Coordinate receiver (main logic) ==========

    def coordinate_receiver(self, msg: Float32MultiArray):
        if len(msg.data) < 9:
            self.get_logger().error(
                "Expected 9 floats [x1,y1,z1,w1,x2,y2,z2,w2,mode]"
            )
            return

        x1, y1, z1, w1, x2, y2, z2, w2, mode = map(float, msg.data[:9])
        mode_int = int(mode)

        self.get_logger().info(
            f"Received: Pt1=({x1:.3f},{y1:.3f},{z1:.3f},{w1:.2f}), "
            f"Pt2=({x2:.3f},{y2:.3f},{z2:.3f},{w2:.2f}), mode={mode_int}"
        )

        # small wait to ensure no old goal is in progress
        self.wait_for_pose()

        if mode_int == 0:
            # Sleep
            self.get_logger().info("Mode 0: Sleep")
            self.send_gr_pose(False)
            self.wait_for_pose()
            self.send_pose(x1, y1, 0.20, w1, mode_int)
            self.wait_for_pose()
            self.send_pose(x1, y1, z1, w1, mode_int)

        elif mode_int == 1:
            # Standby
            self.get_logger().info("Mode 1: Standby")
            self.send_gr_pose(False)
            self.wait_for_pose()
            self.send_pose(x1, y1, z1, w1, mode_int)

        elif mode_int == 2:
            # Take picture only (no picking)
            self.get_logger().info("Mode 2: Picture-only pose")
            self.send_gr_pose(False)
            self.wait_for_pose()
            self.send_pose(x1, y1, z1, w1, mode_int)
            self.wait_for_pose()

            pose_msg = Float32MultiArray()
            pose_msg.data = [x1, y1, z1, w1, float(mode_int)]
            self.pose_reached_publisher.publish(pose_msg)
            self.get_logger().info("Notified vision: pose_reached (mode=2).")

        elif mode_int == 3:
            # Collect & stack cubes:
            # 1) move to camera pose (x1,y1,z1)
            # 2) send pose_reached with stack base (x2,y2,z2)
            self.get_logger().info("Mode 3: Collect & stack via vision.")
            self.send_gr_pose(False)
            self.wait_for_pose()
            self.send_pose(x1, y1, z1, w1, mode_int)
            self.wait_for_pose()

            pose_msg = Float32MultiArray()
            # vision will interpret this as stack base
            pose_msg.data = [x2, y2, z2, w2, float(mode_int)]
            self.pose_reached_publisher.publish(pose_msg)
            self.get_logger().info("Notified vision: pose_reached (mode=3).")

        else:
            # Generic pick & place (mode 4), used by vision for each cube
            self.get_logger().info("Mode 4: Pick & place from coordinates.")
            self.pick_and_place_sequence(x1, y1, z1, w1, x2, y2, z2, w2, mode_int)

    # ========== Pick & Place sequence ==========

    def pick_and_place_sequence(self, x1, y1, z1, w1,
                                x2, y2, z2, w2,
                                mode_int):
        hover = 0.07  # 7 cm hover height

        pick_z_safe = max(z1, 0.03)
        place_z_safe = max(z2, 0.03)

        # 1. Open gripper
        self.send_gr_pose(False)
        self.wait_for_pose()


        # 2. Move above pick (offset)
        self.send_pose(x1, y1, pick_z_safe + hover, w1, mode_int)
        self.wait_for_pose()

        # 3. Move down to pick (offset)
        self.send_pose(x1, y1, pick_z_safe, w1, mode_int)
        self.wait_for_pose()
        # 4. Close gripper
        self.send_gr_pose(True)
        self.wait_for_pose()

        # 5. Lift cube
        self.send_pose(x1, y1, pick_z_safe + hover, w1, mode_int)
        self.wait_for_pose()

        # 6. Move above place
        self.send_pose(x2, y2, place_z_safe + hover, w2, mode_int)
        self.wait_for_pose()

        # 7. Move down to place
        self.send_pose(x2, y2, place_z_safe, w2, mode_int)
        self.wait_for_pose()

        # 8. Open gripper (release)
        self.send_gr_pose(False)
        self.wait_for_pose()

        # 9. Lift up 15 cm before returning
        lift_after_place = place_z_safe + 0.15
        self.send_pose(x2, y2, lift_after_place, w2, mode_int)
        self.wait_for_pose()
        
    # ========== Motion commands ==========

    def send_pose(self, x: float, y: float, z: float, w: float = 1.0, mode=1):
        is_sing, m = self._check_singularity_and_log()
        if is_sing:
            self.get_logger().error(
                f"Aborting send_pose({x:.3f},{y:.3f},{z:.3f}) due to singularity (m={m:.8e})"
            )
            return False

        pose = PoseStamped()
        pose.header.frame_id = self.ref_frame

        
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        yaw = float(math.atan2(y, x))
        roll = 0.0

        dist_xy = math.hypot(x, y)
        if int(mode) == 0:
            pitch = 0.53  # sleep angled down
        elif int(mode) in [1, 2]:
            # typical slightly down angle
            pitch = 0.523
        elif int(mode) in [3, 4]:
            # typical slightly down angle
            pitch = 1.3
        else:
            pitch = 0.0

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        norm_q = np.linalg.norm(q)
        if norm_q < 1e-12:
            q = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            q = q / norm_q


        pose.pose.orientation = Quaternion(
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])
        )

        req = MotionPlanRequest()
        req.group_name = self.group_name_arm
        req.allowed_planning_time = 7.0
        req.num_planning_attempts = 3

        pc = PositionConstraint()
        pc.header.frame_id = self.ref_frame
        pc.link_name = self.ee_link

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]

        oc = OrientationConstraint()
        oc.header.frame_id = self.ref_frame
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 0.5

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pc]
        goal_constraints.orientation_constraints = [oc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        with self.goal_lock:
            self.goal_in_progress = True

        send_future = self._client.send_goal_async(goal,
                                                   feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)
        return True

    def send_gr_pose(self, open: bool = True):
        is_sing, m = self._check_singularity_and_log()
        if is_sing:
            self.get_logger().warn(
                f"Near singularity (m={m:.8e}) while moving gripper; continuing."
            )

        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 2

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.0235 if open else 0.055
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [jc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False

        with self.goal_lock:
            self.goal_in_progress = True

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    # ========== MoveIt callbacks & wait_for_pose ==========

    def wait_for_pose(self, timeout=6.0):
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.05)
            with self.goal_lock:
                if not self.goal_in_progress:
                    return True
        self.get_logger().error(f"wait_for_pose TIMEOUT after {timeout}s, unlocking.")
        with self.goal_lock:
            self.goal_in_progress = False
        return False

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected!')
            with self.goal_lock:
                self.goal_in_progress = False
            return
        self.get_logger().info('MoveIt goal accepted.')
        self.current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().info(f"[Feedback] Current state: {state}")

    def _result_cb(self, future):
        try:
            result = future.result().result
            code = getattr(result.error_code, 'val', -1)
            self.get_logger().info(f"[Result] MoveIt returned error_code: {code}")
        except Exception as e:
            self.get_logger().error(f"Result callback exception: {e}")
        with self.goal_lock:
            self.goal_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = MoveItEEClient(singularity_threshold=1e-5)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MoveItEEClient...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()