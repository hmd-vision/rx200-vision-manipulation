
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from geometry_msgs.msg import Pose, Quaternion
import time

# Services
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState


class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')

        # Cartesian path service client
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        while not self._cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for compute_cartesian_path service...')

        # MoveGroup action client for execution
        self._exec_client = ActionClient(self, MoveGroup, '/move_action')
        while not self._exec_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveGroup action server...')

        self.group_name_arm = 'interbotix_arm'
        self.group_name_gripper = 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/baselink'
        self.gripper_joint = 'left_finger'

        self.declare_parameter('start_state_gripper', value=True)
        self.send_gr_pose(self.get_parameter('start_state_gripper').value)
        self.get_logger().info('Node initialized successfully!')

    def send_pose(self, x, y, z, w=1.0):
        # Create waypoint
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=w)
        waypoints = [pose]

        # Call GetCartesianPath service
        req = GetCartesianPath.Request()
        req.start_state = RobotState()
        req.waypoints = waypoints
        req.max_step = 0.75  # 0.01
        req.jump_threshold = 0.2  # 0.0
        req.avoid_collisions = False
        req.group_name = self.group_name_arm

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.fraction < 1.0:
            self.get_logger().warning(f"Only {response.fraction*100:.1f}% of path planned")

        trajectory = response.solution

        # MoveGroup goal for execution
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name_arm

        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.look_around = False
        goal.request.trajectory = trajectory

        send_future = self._exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected')
            return

        # Wait for execution result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        code = getattr(result.error_code, 'val', 'unknown')
        self.get_logger().info(f'MoveIt execution finished with error code {code}')

    def send_ee_pose(self, x, y, z, w=1.0):
        pass  # Optional placeholder

    def send_gr_pose(self, open=True):
        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.0 if open else 0.035
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [jc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        send_future = self._exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper MoveIt goal rejected')
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Gripper goal executed')


def main():
    rclpy.init()
    node = MoveItEEClient()

    node.send_gr_pose(True)

    node.send_pose(0.3, 0.0, 0.10)
    time.sleep(2.0)
    node.send_pose(0.16203, 0.0, 0.20883)
    time.sleep(2.0)
    node.send_pose(0.066506, 0.3, 0.42362)

    # try:
    #     node.send_pose(0.3, 0.0, 0.10)
    # except Exception as e:
    #     print(f"Skipped Pose 1 cause {e}")
    # 

    # try:
    #     node.send_pose(0.3, 0.2, 0.30)
    # except Exception as e:
    #     print(f"Skipped Pose 2 cause {e}")

    # time.sleep(2.0)
    # try:
    #     node.send_pose(0.2, 0.3, 0.40)
    # except Exception as e:
    #     print(f"Skipped Pose 3 cause {e}")

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


'''
Very Old
'''

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion


class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')

        self._client = ActionClient(self, MoveGroup, '/move_action')
        while not self._client.wait_for_server(1.0):
            self.get_logger().warning('Waiting for action server...')
        self.group_name_arm = 'interbotix_arm'
        self.group_name_gripper = 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/baselink'
        self.gripper_joint = 'left_finger'

        self.declare_parameter('start_state_gripper', value=True)
        self.send_gr_pose(self.get_parameter('start_state_gripper').value)
        self.get_logger().info('Node initialized successfully!')

    def send_pose(self, x, y, z, w=1.0):
        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # pose.pose._orientation.w = w
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=w)

        # tell arm to move to pos
        req = MotionPlanRequest()
        req.group_name = self.group_name_arm
        req.allowed_planning_time = 5.0  # sec
        req.num_planning_attempts = 3

        pc = PositionConstraint()
        pc.header.frame_id=self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE

        # if your robot is behaving weirdly, tune this and some other stuff
        sp.dimensions = [0.01]
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]

        # more for second assignment
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation

        # tune for second assignment
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0  # will be surprised if we have to tune this

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pc]
        goal_constraints.orientation_constraints = [oc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)

        send_future.add_done_callback(self._goal_response_cb)

    def send_ee_pose(self, x, y, z, w=1.0):
        pass

    def send_gr_pose(self, open = True):
        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc= JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.0 if open else 0.035
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [jc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        # Add this to main: node.send_gr_pose(True)
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'MoveIt goal rejected')
            return
        self.get_logger().info('MoveIt goal accepted')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().info(f'[Feedback] {state}')

    def _result_cb(self, future):
        result = future.result().result
        code = getattr(result.error_code, 'val', 'unknown')
        self.get_logger().info(f'[Result] error_code {code}')


def main():
    rclpy.init()
    node = MoveItEEClient()
    # node.send_pose(0.25, 0.0, 0.15)  # single EE pose
    node.send_gr_pose(True)

    node.send_pose(0.3, 0.0, 0.10)
    # node.send_pose(0.4, 0.0, 0.50)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()