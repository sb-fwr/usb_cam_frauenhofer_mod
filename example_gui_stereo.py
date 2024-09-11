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

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.compressed_subscription = self.create_subscription(
            sensor_msgs.msg.CompressedImage,
            '/image0/compressed',
            self.compressed_listener_0_callback,
            1)
        self.compressed_subscription = self.create_subscription(
            sensor_msgs.msg.CompressedImage,
            '/image1/compressed',
            self.compressed_listener_1_callback,
            1)
        self.raw_subscription_0 = self.create_subscription(
            sensor_msgs.msg.Image,
            '/image_0',
            self.raw_listener_callback_0,
            1)
        self.raw_subscription_1 = self.create_subscription(
            sensor_msgs.msg.Image,
            '/image_1',
            self.raw_listener_callback_1,
            1)

        self.latest_image = [None, None]
        self.image_size = (0, 0)
        self.cli = self.create_client(SetBool, '/set_capture')
        self.capture_enabled = True
        self.trigger_mode = False  # Added for trigger mode
        self.declare_parameter('brightness', 0)  # Default value for brightness
        self.display_raw = True

    def compressed_listener_0_callback(self, msg):
        if not self.display_raw:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_image[0] = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if self.latest_image[0] is not None:
                self.image_size = self.latest_image[0].shape[1], self.latest_image[0].shape[0]  # width, height

    def compressed_listener_1_callback(self, msg):
        if not self.display_raw:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_image[1] = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if self.latest_image[1] is not None:
                self.image_size = self.latest_image[1].shape[1], self.latest_image[1].shape[0]  # width, height

    def raw_listener_callback_0(self, msg):
        if self.display_raw:
            print("im0")
            np_arr = np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.width, 2))
            # np_arr = np.ndarray((msg.height, msg.width, 2), dtype=np.uint8, buffer=memoryview(msg.data))
            self.latest_image[0] = cv2.cvtColor(np_arr, cv2.COLOR_YUV2RGB_Y422)
            if self.latest_image[0] is not None:
                self.image_size = self.latest_image[0].shape[1], self.latest_image[0].shape[0]  # width, height

    def raw_listener_callback_1(self, msg):
        if self.display_raw:
            print("im1")
            np_arr = np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.width, 2))
            # np_arr = np.ndarray((msg.height, msg.width, 2), dtype=np.uint8, buffer=memoryview(msg.data))
            self.latest_image[1] = cv2.cvtColor(np_arr, cv2.COLOR_YUV2RGB_Y422)
            if self.latest_image[1] is not None:
                self.image_size = self.latest_image[1].shape[1], self.latest_image[1].shape[0]  # width, height


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

def update_image(label, node, index, scale_var, root):
    if node.latest_image[index] is not None:
        img_pil = Image.fromarray(node.latest_image[index])

        if scale_var.get():
            # Get the dimensions of the label
            window_width = max(root.winfo_width(), 1)
            height = max(label.winfo_height(), 1)
            if(window_width%2):
                width = int((window_width-1)/2)
            else:
                width = int(window_width/2)

            # Get original image dimensions
            original_width, original_height = img_pil.size
            if(original_height > height or original_width > width):
                # Calculate the aspect ratio of the image
                aspect_ratio = original_width/original_height
                # Scale based on the smaller dimension to preserve the aspect ratio
                if width / height > aspect_ratio:
                    # Height is the limiting factor
                    new_height = height
                    new_width = int(height * aspect_ratio)
                else:
                    # Width is the limiting factor
                    new_width = width
                    new_height = int(width / aspect_ratio)

                # Resize the image while keeping aspect ratio
                img_pil = img_pil.resize((new_width, new_height), Image.Resampling.LANCZOS)
            root.resizable(True, True)
        else:
            # Adjust window size to the original image size, add width of buttons
            root.geometry(f"{node.image_size[0]*2}x{node.image_size[1]+130}")
            root.resizable(False, False)
        
        img_tk = ImageTk.PhotoImage(image=img_pil)
        label.config(image=img_tk)
        label.image = img_tk
    label.after(30, update_image, label, node, index, scale_var, root)

def toggle_scale(scale_var):
    scale_var.set(not scale_var.get())

def set_brightness(node, brightness_var):
    try:
        brightness = int(brightness_var.get())
        node.get_logger().info(f"Setting brightness to {brightness} on /usb_cam")
        # Implementation for setting brightness (if applicable)
    except ValueError:
        node.get_logger().error("Invalid brightness value. Please enter a valid integer.")

def main(args=None):
    rclpy.init(args=args)
    root = Tk()
    root.title("ROS2 Compressed Image Viewer")

    # Set initial window size
    root.geometry("800x600")

    scale_var = BooleanVar(value=True)

    # Configure the grid layout for two frames side by side
    root.grid_rowconfigure(0, weight=1)  # Frame row
    root.grid_columnconfigure(0, weight=1)  # Left frame column
    root.grid_columnconfigure(1, weight=1)  # Right frame column

    # Create frames
    left_frame = Frame(root)
    left_frame.grid(row=0, column=0, sticky='nsew')

    right_frame = Frame(root)
    right_frame.grid(row=0, column=1, sticky='nsew')

    # Configure frames to expand
    left_frame.grid_rowconfigure(0, weight=1)  # Image row
    left_frame.grid_rowconfigure(1, weight=0)
    left_frame.grid_rowconfigure(2, weight=0)
    left_frame.grid_rowconfigure(3, weight=0)
    left_frame.grid_columnconfigure(0, weight=1)

    right_frame.grid_rowconfigure(0, weight=1)  # Image row
    right_frame.grid_rowconfigure(1, weight=0)
    right_frame.grid_rowconfigure(2, weight=0)
    right_frame.grid_rowconfigure(3, weight=0)
    right_frame.grid_columnconfigure(0, weight=1)

    # Create labels to display images
    label_0 = Label(left_frame)
    label_0.grid(row=0, column=0, sticky="nsew")

    label_1 = Label(right_frame)
    label_1.grid(row=0, column=0, sticky="nsew")

    image_subscriber = ImageSubscriber()

    # Under left image
    button_scale = Button(
        left_frame, text="Toggle Scale",
        command=lambda: toggle_scale(scale_var), width=15)
    button_scale.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

    brightness_var = StringVar(value='0')
    brightness_entry = Entry(
        left_frame, textvariable=brightness_var)
    brightness_entry.grid(row=2, column=0, sticky="nsew", padx=5, pady=9)

    button_set_brightness = Button(
        left_frame, text="Set Brightness",
        command=lambda: set_brightness(image_subscriber, brightness_var), width=15)
    button_set_brightness.grid(row=3, column=0, sticky="nsew", padx=5, pady=5)

    # Under right image
    button_service = Button(
        right_frame, text="Toggle Capture",
        command=image_subscriber.toggle_capture, width=15)
    button_service.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

    button_toggle_display = Button(
        right_frame, text="Toggle Raw/Compressed",
        command=image_subscriber.toggle_display_mode, width=15)
    button_toggle_display.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)

    button_trigger_mode = Button(
        right_frame, text="Toggle Trigger Mode",
        command=image_subscriber.toggle_trigger_mode, width=15)
    button_trigger_mode.grid(row=3, column=0, sticky="nsew", padx=5, pady=5)

    # ROS 2 Executor for managing ROS callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(image_subscriber)

    # Start ROS spinning in a separate thread
    def spin():
        while rclpy.ok():
            rclpy.spin_once(image_subscriber, timeout_sec=0.1)

    ros_thread = Thread(target=spin)
    ros_thread.start()

    # Update the displayed images in the GUI
    label_0.after(33, update_image, label_0, image_subscriber, 0, scale_var, root)
    label_1.after(33, update_image, label_1, image_subscriber, 1, scale_var, root)
    root.mainloop()

    # Clean up ROS executor and shutdown
    executor.shutdown()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
