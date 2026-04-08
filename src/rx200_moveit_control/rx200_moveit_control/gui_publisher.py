#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import tkinter as tk
from tkinter import messagebox


class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('keyboard_gui')
        self.publisher = self.create_publisher(
            Float32MultiArray, 'goal_coordinates', 10
        )

    def publish_coordinates(self, x1, y1, z1, w1,
                            x2, y2, z2, w2,
                            mode):
        msg = Float32MultiArray()
        msg.data = [
            float(x1), float(y1), float(z1), float(w1),
            float(x2), float(y2), float(z2), float(w2),
            float(mode)
        ]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published coordinates: {msg.data}")


class tkinterGUI:
    def __init__(self, ros_node: CoordinatePublisher):
        self.ros_node = ros_node
        self.root = tk.Tk()
        self.root.title("RX-200 Control GUI")

        # Column headers
        tk.Label(self.root, text="x").grid(row=0, column=1)
        tk.Label(self.root, text="y").grid(row=0, column=2)
        tk.Label(self.root, text="z").grid(row=0, column=3)
        tk.Label(self.root, text="w").grid(row=0, column=4)

        # Pt.1 row
        tk.Label(self.root, text="Pt. 1").grid(row=1, column=0)
        self.entry_x1 = tk.Entry(self.root)
        self.entry_y1 = tk.Entry(self.root)
        self.entry_z1 = tk.Entry(self.root)
        self.entry_w1 = tk.Entry(self.root)

        self.entry_x1.insert(0, "0.3")
        self.entry_y1.insert(0, "0.0")
        self.entry_z1.insert(0, "0.2")
        self.entry_w1.insert(0, "1.0")

        self.entry_x1.grid(row=1, column=1)
        self.entry_y1.grid(row=1, column=2)
        self.entry_z1.grid(row=1, column=3)
        self.entry_w1.grid(row=1, column=4)

        # Pt.2 row
        tk.Label(self.root, text="Pt. 2").grid(row=2, column=0)
        self.entry_x2 = tk.Entry(self.root)
        self.entry_y2 = tk.Entry(self.root)
        self.entry_z2 = tk.Entry(self.root)
        self.entry_w2 = tk.Entry(self.root)

        self.entry_x2.insert(0, "0.4")
        self.entry_y2.insert(0, "0.0")
        self.entry_z2.insert(0, "0.2")
        self.entry_w2.insert(0, "1.0")

        self.entry_x2.grid(row=2, column=1)
        self.entry_y2.grid(row=2, column=2)
        self.entry_z2.grid(row=2, column=3)
        self.entry_w2.grid(row=2, column=4)

        # Cubeville (stack location)
        cube_to_loc_row = 5
        tk.Label(self.root, text="Cubeville").grid(row=cube_to_loc_row, column=0)
        self.entry_x3 = tk.Entry(self.root)
        self.entry_y3 = tk.Entry(self.root)
        self.entry_z3 = tk.Entry(self.root)
        self.entry_w3 = tk.Entry(self.root)

        self.entry_x3.insert(0, "0.10")
        self.entry_y3.insert(0, "-0.20")
        self.entry_z3.insert(0, "0.02")
        self.entry_w3.insert(0, "1.0")

        self.entry_x3.grid(row=cube_to_loc_row, column=1)
        self.entry_y3.grid(row=cube_to_loc_row, column=2)
        self.entry_z3.grid(row=cube_to_loc_row, column=3)
        self.entry_w3.grid(row=cube_to_loc_row, column=4)

        # Buttons
        tk.Button(self.root, text="Go to Sleep",
                  command=self.go_to_sleep).grid(row=3, column=1)
        tk.Button(self.root, text="Go to Standby",
                  command=self.go_to_standby).grid(row=3, column=2)
        tk.Button(self.root, text="Send Coords",
                  command=self.send_coordinates).grid(row=3, column=3)

        # --- OLD BUTTONS REMOVED ---
        # Take Picture
        # Collect Cubes

        # --- NEW MERGED BUTTON ---
        tk.Button(self.root,
                  text="Collect & Stack Cubes",
                  command=self.take_picture_and_collect
                 ).grid(row=7, column=1, columnspan=3)

    # --- workspace checks ---

    def reachable(self, x, y, z, w=1.0):
        x = float(x); y = float(y); z = float(z)
        dist = (x**2 + y**2 + z**2) ** 0.5
        if dist >= 0.55:
            return False, 'Beyond reach of RX-200.'

        dist_xy = (x**2 + y**2) ** 0.5
        if (dist_xy <= 0.07) and (z <= 0.07):
            return False, 'Point inside robot base.'

        if (z <= 0.05) and (0 >= x >= -0.2) and (abs(y) <= 0.05):
            return False, 'Point inside utilities area.'
        return True, 'OK'

    # --- button callbacks ---

    def go_to_sleep(self):
        self.ros_node.publish_coordinates(
            0.12175, 0.0, 0.08, 1.0,
            0.12175, 0.0, 0.08, 1.0,
            0.0
        )

    def go_to_standby(self):
        self.ros_node.publish_coordinates(
            0.12175, 0.0, 0.20, 1.0,
            0.12175, 0.0, 0.20, 1.0,
            1.0
        )

    # ========== NEW MERGED FUNCTION ==========
    def take_picture_and_collect(self):
        # --- Step 1: Move to picture pose (Mode 2) ---
        self.ros_node.publish_coordinates(
            0.12, 0.0, 0.35, 1.0,     # picture pose
            0.0, 0.0, 0.0, 1.0,       # dummy Pt2
            2.0                       # mode 2: take picture
        )

        # --- Step 2: Immediately send collect command (Mode 3) ---
        x3 = self.entry_x3.get()
        y3 = self.entry_y3.get()
        z3 = self.entry_z3.get()
        w3 = self.entry_w3.get()

        ok, err = self.reachable(x3, y3, z3)
        if not ok:
            messagebox.showerror("Error", f"Cubeville {err}")
            return

        self.ros_node.publish_coordinates(
            0.12, 0.0, 0.35, 1.0,   # camera pose for mode 3
            x3, y3, z3, w3,         # stacking base
            3.0                     # mode 3: collect & stack
        )

    def send_coordinates(self):
        x1 = self.entry_x1.get()
        y1 = self.entry_y1.get()
        z1 = self.entry_z1.get()
        w1 = self.entry_w1.get()

        x2 = self.entry_x2.get()
        y2 = self.entry_y2.get()
        z2 = self.entry_z2.get()
        w2 = self.entry_w2.get()

        ok1, err1 = self.reachable(x1, y1, z1)
        ok2, err2 = self.reachable(x2, y2, z2)

        if not ok1:
            messagebox.showerror("Error", f"Pt.1: {err1}")
            return
        if not ok2:
            messagebox.showerror("Error", f"Pt.2: {err2}")
            return

        self.ros_node.publish_coordinates(
            x1, y1, z1, w1,
            x2, y2, z2, w2,
            4.0
        )

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    ros_node = CoordinatePublisher()

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True
    )
    ros_thread.start()

    gui = tkinterGUI(ros_node)
    gui.run()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
