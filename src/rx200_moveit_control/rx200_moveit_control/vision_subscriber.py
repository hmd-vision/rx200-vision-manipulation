#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class CubeDetector:
    def __init__(self, camera_info: CameraInfo):
        self.min_area = 800
        self.max_area = 80000
        self.min_box_frac = 0.003
        self.max_box_frac = 0.10

        self.min_rectangularity = 0.65
        self.min_solidity = 0.90
        self.aspect_ratio_tol = 0.35

        self.COLOR_RANGES = {
            "red": [
                ((0, 70, 60), (10, 255, 255)),
                ((170, 70, 60), (179, 255, 255))
            ],
            "blue": [
                ((90, 40, 40), (140, 255, 255))
            ],
            "yellow": [
                ((15, 60, 80), (40, 255, 255))
            ]
        }

        self.fx = camera_info.k[0]
        self.fy = camera_info.k[4]
        self.cx = camera_info.k[2]
        self.cy = camera_info.k[5]

        self.K = np.array(
            [[self.fx, 0.0, self.cx],
             [0.0, self.fy, self.cy],
             [0.0, 0.0, 1.0]],
            dtype=np.float32
        )

        self.T_cam_to_robot = None

    def depth_distance_m(self, depth_img, cx, cy):
        if depth_img is None:
            return None
        h, w = depth_img.shape
        x0, x1 = max(cx - 2, 0), min(cx + 3, w)
        y0, y1 = max(cy - 2, 0), min(cy + 3, h)
        patch = depth_img[y0:y1, x0:x1]
        vals = patch[patch > 0]
        if vals.size == 0:
            return None
        return float(np.median(vals)) / 1000.0

    def backproject(self, u, v, Z):
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return np.array([X, Y, Z], dtype=np.float32)

    def transform_cam_to_robot(self, p_cam):
        if self.T_cam_to_robot is None:
            return None
        p_h = np.array([*p_cam, 1.0], dtype=np.float32)
        p_r = self.T_cam_to_robot @ p_h
        return p_r[:3]

    def detect(self, color_img, depth_img):
        hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        annotated = color_img.copy()
        cubes = []

        H, W = hsv.shape[:2]
        img_area = H * W
        min_box_area = self.min_box_frac * img_area
        max_box_area = self.max_box_frac * img_area

        for color_name, ranges in self.COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))

            kernel = np.ones((7, 7), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, 2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, 2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (self.min_area < area < self.max_area):
                    continue

                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
                if len(approx) != 4 or not cv2.isContourConvex(approx):
                    continue

                x, y, w, h = cv2.boundingRect(approx)
                rect_area = w * h
                if not (min_box_area < rect_area < max_box_area):
                    continue

                aspect = w / float(h)
                if abs(aspect - 1.0) > self.aspect_ratio_tol:
                    continue

                rectangularity = area / rect_area
                if rectangularity < self.min_rectangularity:
                    continue

                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                if hull_area <= 0:
                    continue

                solidity = area / hull_area
                if solidity < self.min_solidity:
                    continue

                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                Z = self.depth_distance_m(depth_img, cx, cy)
                if Z is None or not (0.10 < Z < 1.2):
                    continue

                center_cam = self.backproject(cx, cy, Z)
                center_robot = self.transform_cam_to_robot(center_cam)
                distance_cm = Z * 100.0

                box = cv2.boxPoints(cv2.minAreaRect(cnt))
                box = np.int32(box)
                cv2.drawContours(annotated, [box], 0, (0, 255, 0), 2)
                cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(annotated, f"{color_name} ({distance_cm:.1f} cm)",
                            (x, max(0, y - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 2)

                cubes.append({
                    "color": color_name,
                    "cx": cx,
                    "cy": cy,
                    "distance_cm": distance_cm,
                    "center_cam": center_cam,
                    "center_robot": center_robot
                })

        return annotated, cubes


class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision')

        self.bridge = CvBridge()
        self.detector = None
        self.camera_info_received = False

        self.current_color_image = None
        self.current_depth_image = None

        self.last_moveit_done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_sub = self.create_subscription(Float32MultiArray, 'pose_reached',
                                                 self.pose_reached_callback, 10)
        self.moveit_done_sub = self.create_subscription(Float32MultiArray, 'moveit_done',
                                                        self.moveit_done_callback, 10)

        self.camera_info_sub = self.create_subscription(CameraInfo,
                                                        '/camera/camera/color/camera_info',
                                                        self.camera_info_callback, 10)

        self.color_sub = self.create_subscription(Image,
                                                  '/camera/camera/color/image_raw',
                                                  self.color_callback, 10)

        self.depth_sub = self.create_subscription(Image,
                                                  '/camera/camera/depth/image_rect_raw',
                                                  self.depth_callback, 10)

        self.goal_pub = self.create_publisher(Float32MultiArray, 'goal_coordinates', 10)

        self.get_logger().info("Vision node initialized.")


    def moveit_done_callback(self, msg):
        """Triggered when MoveIt finishes a goal."""
        self.last_moveit_done = True


    def get_transform_matrix(self, target_frame, source_frame):
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                                    rclpy.time.Time())

            t = trans.transform.translation
            q = trans.transform.rotation

            R = np.array([
                [1 - 2*(q.y*q.y + q.z*q.z), 2*(q.x*q.y - q.z*q.w), 2*(q.x*q.z + q.y*q.w)],
                [2*(q.x*q.y + q.z*q.w), 1 - 2*(q.x*q.x + q.z*q.z), 2*(q.y*q.z - q.x*q.w)],
                [2*(q.x*q.z - q.y*q.w), 2*(q.y*q.z + q.x*q.w), 1 - 2*(q.x*q.x + q.y*q.y)]
            ], dtype=np.float32)

            T = np.eye(4, dtype=np.float32)
            T[:3, :3] = R
            T[:3, 3] = [t.x, t.y, t.z]
            return T

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None


    def ensure_cam_to_robot_tf(self):
        if self.detector is None:
            return False
        if self.detector.T_cam_to_robot is not None:
            return True

        target = 'rx200/base_link'
        candidates = [
            'rx200/camera_depth_optical_frame',
            'rx200/camera_color_optical_frame',
            'camera_depth_optical_frame',
            'camera_color_optical_frame',
        ]

        for src in candidates:
            T = self.get_transform_matrix(target, src)
            if T is not None:
                self.detector.T_cam_to_robot = T
                return True

        self.get_logger().error("Failed to acquire camera→robot TF.")
        return False


    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.detector = CubeDetector(msg)
            self.camera_info_received = True
            self.get_logger().info("Camera intrinsics loaded.")

        self.ensure_cam_to_robot_tf()


    def color_callback(self, msg):
        try:
            self.current_color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Color conversion failed: {e}")


    def depth_callback(self, msg):
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")


    def wait_for_moveit(self, timeout=10.0):
        """Waits for MoveIt to signal completion."""
        self.last_moveit_done = False
        start = time.time()

        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_moveit_done:
                return True

        self.get_logger().warn("MoveIt did not confirm completion.")
        return False


    def pose_reached_callback(self, msg):
        data = list(msg.data)
        if len(data) < 5:
            return

        x, y, z, w, mode = data[:5]
        mode = int(mode)

        if not self.detector or not self.camera_info_received:
            return
        if self.current_color_image is None or self.current_depth_image is None:
            return

        self.ensure_cam_to_robot_tf()

        color = self.current_color_image.copy()
        depth = self.current_depth_image.copy()

        annotated, cubes = self.detector.detect(color, depth)

        if mode == 2:
            ts = time.strftime("%m%d_%H%M%S")
            filename = f"annotated_{ts}.png"
            cv2.imwrite(filename, annotated)
            self.get_logger().info(f"Saved annotated image: {filename}")

        if mode == 2:
            return

        if mode == 3:
            if len(cubes) == 0:
                self.get_logger().info("No cubes detected.")
                return

            cubes_sorted = sorted(cubes, key=lambda c: c["distance_cm"])
            base_x, base_y, base_z, base_w = x, y, z, w

            cube_height = 0.05
            max_cubes = min(3, len(cubes_sorted))

            for i in range(max_cubes):
                c = cubes_sorted[i]
                cr = c["center_robot"]
                if cr is None:
                    continue

                pick_x, pick_y, pick_z = map(float, cr)
                pick_z = max(pick_z, 0.02)

                place_z = base_z + i * cube_height
                place_z = max(place_z, 0.02)

                msg_out = Float32MultiArray()
                msg_out.data = [
                    pick_x, pick_y, pick_z, 1.0,
                    base_x, base_y, place_z, 1.0,
                    4.0
                ]
                self.goal_pub.publish(msg_out)
                time.sleep(0.3)

                self.wait_for_moveit()

            '''# After all cubes are done, return to standby pose
            standby_msg = Float32MultiArray()
            standby_msg.data = [
                0.12175, 0.0, 0.20, 1.0,
                0.12175, 0.0, 0.20, 1.0,
                1.0
            ]
            self.goal_pub.publish(standby_msg)
            self.get_logger().info("AUTO-STANDBY executed.")'''


def main(args=None):
    rclpy.init(args=args)
    node = VisionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()