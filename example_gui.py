import sys
import rclpy
from rclpy.node import Node
import sensor_msgs.msg
from std_srvs.srv import SetBool
import numpy as np
import cv2
from tkinter import Tk, Label, Button, BooleanVar, Frame
from PIL import Image, ImageTk
from threading import Thread
from tkinter import Entry, StringVar
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
# from builtin_interfaces.msg import Duration
from rclpy.duration import Duration

qos = QoSProfile(
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    liveliness=QoSLivelinessPolicy.AUTOMATIC,
    deadline=Duration(seconds=0),  # Disable deadline
    lifespan=Duration(seconds=1),  # Message lifespan is 1 second
    liveliness_lease_duration=Duration(seconds=0),  # Default liveliness lease duration (infinite)
    avoid_ros_namespace_conventions=False
)


class ImageSubscriber(Node):
    def __init__(self, cam_nr):
        self.cam_nr = cam_nr
        print("runnning cam_nr: ", self.cam_nr)
        super().__init__('image_subscriber')

        # Set the initial timestamp for the last message received
        self.last_msg_time = time.time()
        self.subscribe_to_topics()
        self.latest_image = [None]
        self.image_size = (0, 0)
        self.cli = self.create_client(SetBool, '/set_capture')
        self.capture_enabled = True
        self.trigger_mode = False  # Added for trigger mode
        self.declare_parameter('brightness', 0)  # Default value for brightness
        self.display_raw = True
        # Start a timer to check if we need to restart subscriptions
        self.create_timer(1.0, self.check_message_timeout)

    def subscribe_to_topics(self):
        """Subscribe to the image topics."""
        self.get_logger().info(f"Subscribing to camera {self.cam_nr} topics")
        self.compressed_subscription = self.create_subscription(
            sensor_msgs.msg.CompressedImage,
            '/image_' + str(self.cam_nr) + '/compressed',
            self.compressed_listener_callback,
            qos)
        self.raw_subscription = self.create_subscription(
            sensor_msgs.msg.Image,
            '/image_' + str(self.cam_nr),
            self.raw_listener_callback,
            qos)

    def unsubscribe_from_topics(self):
        """Unsubscribe from the image topics."""
        self.get_logger().info(f"Unsubscribing from camera {self.cam_nr} topics")
        self.destroy_subscription(self.compressed_subscription)
        self.destroy_subscription(self.raw_subscription)

    def check_message_timeout(self):
        """Check if no messages were received in the last 5 seconds."""
        current_time = time.time()
        if current_time - self.last_msg_time > 5.0:
            self.get_logger().info(f"No messages received for 5 seconds. Restarting subscriptions for camera {self.cam_nr}.")
            self.unsubscribe_from_topics()
            time.sleep(1)  # Brief sleep before resubscribing
            self.subscribe_to_topics()
            self.last_msg_time = time.time()

    def compressed_listener_callback(self, msg):
        if not self.display_raw:
            self.last_msg_time = time.time()
            self.np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_image[0] = cv2.imdecode(self.np_arr, cv2.IMREAD_COLOR)
            if self.latest_image[0] is not None:
                self.image_size = self.latest_image[0].shape[1], self.latest_image[0].shape[0]  # width, height

    def raw_listener_callback(self, msg):
        if self.display_raw:
            self.last_msg_time = time.time()
            print("im" + str(self.cam_nr))
            self.np_arr = np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.width, 2))
            self.latest_image[0] = cv2.cvtColor(self.np_arr, cv2.COLOR_YUV2RGB_Y422)
            if self.latest_image[0] is not None:
                self.image_size = self.latest_image[0].shape[1], self.latest_image[0].shape[0]  # width, height

    def toggle_display_mode(self):
        self.display_raw = not self.display_raw
        self.get_logger().info(f'Toggled display mode to {"raw" if self.display_raw else "compressed"}')

    def toggle_capture(self):
        self.capture_enabled = not self.capture_enabled
        req = SetBool.Request()
        req.data = self.capture_enabled
        if self.cli.service_is_ready():
            self.cli.call_async(req)
            self.get_logger().info(f'Set capture to {self.capture_enabled}')
        else:
            self.get_logger().warn('Service not available')

    def toggle_trigger_mode(self):
        self.trigger_mode = not self.trigger_mode
        self.get_logger().info(f"Trigger mode set to {'ON' if self.trigger_mode else 'OFF'}")


def update_image(label, node, scale_var, root):
    if node.latest_image[0] is not None:
        img_pil = Image.fromarray(node.latest_image[0])

        if scale_var.get():
            window_width = max(root.winfo_width(), 1)
            height = max(label.winfo_height(), 1)

            original_width, original_height = img_pil.size
            aspect_ratio = original_width / original_height

            if window_width / height > aspect_ratio:
                new_height = height
                new_width = int(height * aspect_ratio)
            else:
                new_width = window_width
                new_height = int(window_width / aspect_ratio)

            img_pil = img_pil.resize((new_width, new_height), Image.Resampling.LANCZOS)
            root.resizable(True, True)
        else:
            root.geometry(f"{node.image_size[0]}x{node.image_size[1]+130}")
            root.resizable(False, False)

        img_tk = ImageTk.PhotoImage(image=img_pil)
        label.config(image=img_tk)
        label.image = img_tk

    label.after(30, update_image, label, node, scale_var, root)


def toggle_scale(scale_var):
    scale_var.set(not scale_var.get())


def set_brightness(node, brightness_var):
    try:
        brightness = int(brightness_var.get())
        node.get_logger().info(f"Setting brightness to {brightness} on /usb_cam")
    except ValueError:
        node.get_logger().error("Invalid brightness value. Please enter a valid integer.")


def main(args=None):
    rclpy.init(args=args)
        # If arguments are passed from the terminal, sys.argv[1:] contains them
    if args is None:
        args = sys.argv[1:]  # Skipping the first argument (script name)

    root = Tk()
    root.title("ROS2 Image Viewer")

    root.geometry("800x600")

    scale_var = BooleanVar(value=True)

    # Configure the layout to have one frame for the image and one for the buttons
    root.grid_rowconfigure(0, weight=4)  # Image row
    root.grid_rowconfigure(1, weight=1)  # Buttons row
    root.grid_columnconfigure(0, weight=1)

    # Create the frame for the image
    image_frame = Frame(root)
    image_frame.grid(row=0, column=0, sticky='nsew')

    # Configure the frame to expand
    image_frame.grid_rowconfigure(0, weight=1)
    image_frame.grid_columnconfigure(0, weight=1)

    # Create a label to display the image
    label = Label(image_frame)
    label.grid(row=0, column=0, sticky="nsew")

    # Create a frame for the buttons and text fields
    button_frame = Frame(root)
    button_frame.grid(row=1, column=0, sticky='nsew')

    # Configure 3x3 grid for the buttons and text fields
    for i in range(3):
        button_frame.grid_columnconfigure(i, weight=1)
        button_frame.grid_rowconfigure(i, weight=1)

    print(args)
    if len(args) == 0:
        image_subscriber = ImageSubscriber(0)
    else:
        image_subscriber = ImageSubscriber(args[0])

    # 1st row (left to right): Toggle Scale, Brightness Text Field, Set Brightness
    button_scale = Button(
        button_frame, text="Toggle Scale",
        command=lambda: toggle_scale(scale_var), width=15)
    button_scale.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

    brightness_var = StringVar(value='0')
    brightness_entry = Entry(button_frame, textvariable=brightness_var)
    brightness_entry.grid(row=0, column=1, sticky="nsew", padx=5, pady=9)

    button_set_brightness = Button(
        button_frame, text="Set Brightness",
        command=lambda: set_brightness(image_subscriber, brightness_var), width=15)
    button_set_brightness.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)

    # 2nd row (left to right): Toggle Capture, Toggle Raw/Compressed, Toggle Trigger Mode
    button_capture = Button(
        button_frame, text="Toggle Capture",
        command=image_subscriber.toggle_capture, width=15)
    button_capture.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

    button_toggle_display = Button(
        button_frame, text="Toggle Raw/Compressed",
        command=image_subscriber.toggle_display_mode, width=15)
    button_toggle_display.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)

    button_trigger_mode = Button(
        button_frame, text="Toggle Trigger Mode",
        command=image_subscriber.toggle_trigger_mode, width=15)
    button_trigger_mode.grid(row=1, column=2, sticky="nsew", padx=5, pady=5)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(image_subscriber)

    def spin():
        while rclpy.ok():
            rclpy.spin_once(image_subscriber, timeout_sec=0.1)

    ros_thread = Thread(target=spin)
    ros_thread.start()

    label.after(33, update_image, label, image_subscriber, scale_var, root)
    root.mainloop()

    executor.shutdown()
    rclpy.shutdown()
    ros_thread.join()


if __name__ == '__main__':
    main()
