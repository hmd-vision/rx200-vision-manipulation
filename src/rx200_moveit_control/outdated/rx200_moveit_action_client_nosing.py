#!/usr/bin/env python3
"""
moveit_ee_client_combined.py

Combined ROS2 MoveIt client for RX200 with:
- Singularity detection using POE manipulability
- Vision system integration (picture mode, pose verification)
- Enhanced safety
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
    OrientationConstraint, JointConstraint)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float32MultiArray, Bool
import tf_transformations

from geometry_msgs.msg import Point

import time
import numpy as np
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class MoveItEEClient(Node):
    """
    A ROS 2 Node that acts as a client for MoveIt's MoveGroup action server.

    Responsibilities:
    - Send arm movement goals (end-effector poses)
    - Send gripper open/close goals
    - Handle feedback, goal response, and results from MoveIt
    """

    def __init__(self):
        super().__init__('rx200_moveit_control')

        # Create an ActionClient for the MoveGroup action (MoveIt motion planning)
        self._client = ActionClient(self, MoveGroup, '/move_action')

        # Wait until the MoveIt action server is ready
        while not self._client.wait_for_server(1.0):
            self.get_logger().warning('Waiting for MoveIt action server...')

        # Define MoveIt planning groups (as defined in the SRDF - Semantic Robot Description Format)
        self.group_name_arm = 'interbotix_arm'
        self.group_name_gripper = 'interbotix_gripper'

        # Define important robot link and joint names (as used in the URDF - Unified Robot Description Format)
        self.ee_link = 'rx200/ee_gripper_link'
        self.gripper_joint = 'left_finger'        
        self.base_link = 'rx200/base_link'
        self.world = 'world'
        self.ref_frame = self.world  # options: 'world' or 'rx200/base_link'
        
        # Declare a parameter to define the initial gripper state (open or closed)
        self.declare_parameter('start_state_gripper', value=True)

        # Initialize gripper to its start position
        self.send_gr_pose(self.get_parameter('start_state_gripper').value)

        self.get_logger().info('MoveItEEClient Node initialized successfully!')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher to signal vision node that pose is reached
        self.pose_reached_publisher = self.create_publisher(Bool, 'pose_reached', 10)

        # Track if we're waiting for a goal to complete
        self.waiting_for_goal = False
        self.current_goal_handle = None

        # Listen for Coordinates
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'goal_coordinates',
            self.coordinate_receiver,
            10
        )

    def coordinate_receiver(self, msg):
        """Receive Float32MultiArray [x, y, z, w] and call send_pose()."""
        if len(msg.data) < 4:
            self.get_logger().error("Expected 8 floats [x, y, z, w] in message!")
            return

        x1, y1, z1, w1, x2, y2, z2, w2= map(float, msg.data[:8])
        self.get_logger().info(f"Received Pt.1: x={x1}, y={y1}, z={z1}, w={w1}")
        self.get_logger().info(f"Received Pt.2: x={x2}, y={y2}, z={z2}, w={w2}")

        # Small delay helps with scene stability
        time.sleep(0.3)

        if w1 <= 0.2:
            # sleep mode detected
            # Open gripper

            self.send_gr_pose(False)

            # Go to bed

            # TODO: change timeouts to check_pose_reached(self, target_pose, tolerance=0.02)
            time.sleep(0.3)
            self.send_pose(x1, y1, 0.2, w1)
            time.sleep(0.3)
            self.send_pose(x1, y1, z1, w1)
        elif w1 <= 0.4:
            # standby mode detected
            # Open gripper
            self.send_gr_pose(False)

            # go to standby
            time.sleep(0.3)
            self.send_pose(x1, y1, z1, w1)

        elif w1 <= 0.5:
            # Take a picture mode detected
            self.send_gr_pose(False)

            # go to picture pose
            time.sleep(0.3)
            self.send_pose(x1, y1, z1, w1)

            # check that pose has been reached
            self.check_pose_reached([x1, y1, z1], tolerance=0.02)

            # Send signal to vision_subscriber that pose has been reached
            msg = Bool()
            msg.data = True
            self.pose_reached_publisher.publish(msg)
            self.get_logger().info("Say Cheese!")

        else:
            # pick and place mode detected    
            # 1. Open gripper
            self.send_gr_pose(False)
            time.sleep(0.3)
            self.send_pose(x1, y1, z1, w1)
            time.sleep(0.3)
            self.send_gr_pose(True)

            # Don't drag along the ground
            if z1 <=0.07:
                pick_up = z1 + 0.05
                time.sleep(0.3)
                self.send_pose(x1, y1, pick_up, w1)
            else:
                pick_up = z1
            time.sleep(0.3)

            # Dont slam at an angle
            if z2 <= 0.07:
                self.send_pose(x2, y2, pick_up, w2)
                time.sleep(0.3)
                self.send_pose(x2, y2, z2, w2)
            else:
                self.send_pose(x2, y2, z2, w2)

            time.sleep(0.3)
            self.send_gr_pose(False)

    # -------------------------------------------------------------------------
    # Function: send_intermediary_pose
    # -------------------------------------------------------------------------
    def send_pose(self, x, y, z, w=1.0):
        """
        Send an end-effector (EE) position and orientation goal to MoveIt.

        Parameters:
            x, y, z (float): Target position in meters relative to base_link.
            w (float): Quaternion 'w' component (rotation scalar). Defaults to 1.0 (no rotation).

        This function constructs a MoveIt MotionPlanRequest specifying
        both a position and orientation constraint for the end-effector.
        """

        # Define target pose relative to the robot's base frame
        pose = PoseStamped()
        pose.header.frame_id = self.ref_frame  # self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
 
        yaw = np.arctan2(y, x)
        roll = 0.0

        dis_from_base = (y**2 + x**2)**0.5
        if w <= 0.2:
            # sleep mode detected
            pitch = 0.53  # 45 deg
        elif w <= 0.5:
            # picture mode detected
            pitch = 0.523  # 30 deg
        elif dis_from_base < 0.087 and z < 0.19:
            pitch = 1.57  # point down
        else:
            pitch = 0.00  # flat

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        q = q / np.linalg.norm(q) 
        self.get_logger().info(f'Quaternian Values (yaw: {yaw}): x:{q[0]:.3f} y:{q[1]:.3f} z:{q[2]:.3f} w:{q[3]:.3f}')
        pose.pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]),w=float(q[3]))

        # Create a MotionPlanRequest for the arm group
        req = MotionPlanRequest()
        req.group_name = self.group_name_arm
        req.allowed_planning_time = 7.0  # seconds MoveIt is allowed to plan
        req.num_planning_attempts = 3    # number of retries if planning fails

        # -------------------------------
        # Goal Position constraint definition
        # -------------------------------
        pc = PositionConstraint()
        pc.header.frame_id = self.ref_frame  # self.base_link
        pc.link_name = self.ee_link

        # Define the target region as a small sphere
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]  # radius in meters — tune if arm oscillates or misses goal
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]

        # -------------------------------
        # Orientation constraint definition
        # -------------------------------
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05  # roll (radians)
        oc.absolute_y_axis_tolerance = 0.05  # 3.14159  # pitch (radians)
        oc.absolute_z_axis_tolerance = 0.05  # 3.14159  # yaw (radians)
        oc.weight = 0.5  # relative importance of orientation constraint

        # Combine constraints into a single goal constraint object.
        # In MoveIt, a Constraint tells the planner where or how the robot’s 
        # end-effector (or joints) is allowed to be at the goal.
        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pc]
        goal_constraints.orientation_constraints = [oc]
        req.goal_constraints = [goal_constraints]

        # -------------------------------
        # Wrap the request into a MoveGroup goal
        # -------------------------------
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False  # Execute plan once computed
        goal.planning_options.replan = True      # Allow replanning if first attempt fails
        goal.planning_options.look_around = False  # Disable obstacle avoidance head motion

        # Send goal asynchronously and attach callbacks
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def get_current_ee_pose(self):
        try:
            # Lookup the transform from base_link → ee_link
            # transform: TransformStamped = self.tf_buffer.lookup_transform(
            #     self.base_link, self.ee_link, rclpy.time.Time())
            
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.ref_frame, self.ee_link, rclpy.time.Time())
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().error(f'Failed to get current EE pose: {e}')
            return None

    def get_current_ee_pose(self):
        try:
            # Lookup the ee pose          
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.ref_frame, self.ee_link, rclpy.time.Time())
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().error(f'Failed to get current EE pose: {e}')
            return None

    def check_pose_reached(self, target_pose, tolerance=0.02):
        """
        are we there yet?
        """
        current_pose = self.get_current_ee_pose()
        if current_pose is None:
            return False
        
        # Calculate position error
        dx = float(current_pose.pose.position.x) - target_pose[0]
        dy = float(current_pose.pose.position.y) - target_pose[1]
        dz = float(current_pose.pose.position.z) - target_pose[2]
        position_error = np.sqrt(dx**2 + dy**2 + dz**2)
        
        self.get_logger().info(f'Position error: {position_error:.4f}m')
        return position_error < tolerance


    # -------------------------------------------------------------------------
    # Function: send_ee_pose (placeholder for future use)
    # -------------------------------------------------------------------------
    def send_ee_pose(self, x, y, z, w=1.0):
        """Future function for sending end-effector pose using different constraints."""
        pass

    # -------------------------------------------------------------------------
    # Function: send_gr_pose
    # -------------------------------------------------------------------------
    def send_gr_pose(self, open=True):
        """
        Send a gripper coelf.group_name_arm
        req.allowed_planning_time = 5.0  # seconds MoveIt is allowed to plan
        req.num_planning_attempts = 3    # number of retries if planning fails

        # -------------------------------
        # Goal Position command to MoveIt (open or close).

        Parameters:
            open (bool): True to open gripper, False to close.

        This function constructs a MotionPlanRequest that defines
        a joint position constraint for the gripper joint.
        """
        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 2

        # Define the joint constraint for the gripper
        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.0 if open else 0.045  # smaller = open, larger = closed
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0  # importance of this constraint

        # Combine constraint into MoveIt-compatible structure
        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [jc]
        req.goal_constraints = [goal_constraints]

        # Build final MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False  # execute immediately

        # Send the goal to MoveIt
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    # -------------------------------------------------------------------------
    # Callback: Goal response
    # -------------------------------------------------------------------------
    def _goal_response_cb(self, future):
        """
        Callback executed once the goal has been accepted or rejected by MoveIt.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected!')
            self.waiting_for_goal = False
            return
        self.get_logger().info('MoveIt goal accepted.')
        self.current_goal_handle = goal_handle
        # Attach result callback once goal is active
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    # -------------------------------------------------------------------------
    # Callback: Feedback during motion planning/execution
    # -------------------------------------------------------------------------
    def _feedback_cb(self, feedback_msg):
        """
        Callback executed while MoveIt is processing the goal.

        Provides information about current planning state or execution progress.
        """
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().info(f'[Feedback] Current state: {state}')

    # -------------------------------------------------------------------------
    # Callback: Result after goal completion
    # -------------------------------------------------------------------------
    def _result_cb(self, future):
        """
        Callback executed once MoveIt finishes planning/executing the goal.

        Displays the resulting MoveIt error code (0 = SUCCESS).
        """
        result = future.result().result
        code = getattr(result.error_code, 'val', 'unknown')
        self.get_logger().info(f'[Result] MoveIt returned error_code: {code}')


# =============================================================================
# Main entry point
# =============================================================================
def main():
    """
    Entry point for the ROS 2 node.

    Initializes the MoveItEEClient node, sends a series of arm and gripper
    commands, and keeps spinning to process callbacks.
    """
    rclpy.init()
    node = MoveItEEClient()

    # Example usage sequence:
    # 1. Open gripper
    # node.send_gr_pose(False)

    # node.send_pose(-0.2, 0.0, 0.10)
    # node.send_gr_pose(True)

    # node.send_pose(0.3, -0.2, 0.10)

    # # 2. Move to various poses (demo path)
    # node.send_pose(0.5, 0.0, 0.00)
    # node.send_gr_pose(True)

    # # 3. Go here and drop it into space
    # node.send_pose(0.3, 0.0, 0.10)
    # node.send_gr_pose(False)


    # node.send_gr_pose(False)
    # Keep node alive to process results and feedback
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
