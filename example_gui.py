import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import SetBool
import numpy as np
import cv2
from tkinter import Tk, Label, Button, BooleanVar
from PIL import Image, ImageTk
from threading import Thread
from tkinter import Entry, StringVar
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.latest_image = None
        self.image_size = (0, 0)
        self.cli = self.create_client(SetBool, '/set_capture')
        self.capture_enabled = True
        self.declare_parameter('brightness', 0)  # Default value for brightness


    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if self.latest_image is not None:
            self.image_size = self.latest_image.shape[1], self.latest_image.shape[0]  # width, height

    def toggle_capture(self):
        self.capture_enabled = not self.capture_enabled
        req = SetBool.Request()
        req.data = self.capture_enabled
        if self.cli.service_is_ready():
            self.cli.call_async(req)
            self.get_logger().info(f'Set capture to {self.capture_enabled}')
        else:
            self.get_logger().warn('Service not available')

def update_image(label, node, scale_var, root):
    if node.latest_image is not None:
        img_rgb = cv2.cvtColor(node.latest_image, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img_rgb)

        if scale_var.get():
            # Get the dimensions of the Tkinter window
            window_width = max(label.winfo_width(), 1)
            window_height = max(label.winfo_height(), 1)
            # Resize the image to fit the Tkinter window
            img_pil = img_pil.resize((window_width, window_height), Image.Resampling.LANCZOS)
            root.resizable(True, True)
        else:
            # Adjust window size to the original image size
            root.geometry(f"{node.image_size[0]}x{node.image_size[1]}")
            root.resizable(False, False)
        
        img_tk = ImageTk.PhotoImage(image=img_pil)
        label.config(image=img_tk)
        label.image = img_tk
    label.after(100, update_image, label, node, scale_var, root)

def toggle_scale(scale_var):
    scale_var.set(not scale_var.get())

def set_brightness(node, brightness_var):
    try:
        brightness = int(brightness_var.get())
        node.get_logger().info(f"Setting brightness to {brightness} on /usb_cam")

        # Create a parameter client for the /usb_cam node
        parameter_client = node.create_client(SetParameters, '/usb_cam/set_parameters')

        # Wait for the service to become available
        if not parameter_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error("Parameter service not available on /usb_cam")
            return

        # Create a parameter request
        param = Parameter(
            name='brightness',
            value=brightness
        )
        req = SetParameters.Request(parameters=[param.to_parameter_msg()])

        # Send the request to set the parameter
        future = parameter_client.call_async(req)

        # Wait for the result
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            node.get_logger().info(f"Brightness set to {brightness} on /usb_cam")
        else:
            node.get_logger().error(f"Failed to set brightness: {future.exception()}")
    
    except ValueError:
        node.get_logger().error("Invalid brightness value. Please enter a valid integer.")



def main(args=None):
    rclpy.init(args=args)
    root = Tk()
    root.title("ROS2 Compressed Image Viewer")

    # Set initial window size
    root.geometry("800x600")

    scale_var = BooleanVar(value=True)

    # Configure the grid layout
    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)

    label = Label(root)
    label.grid(row=0, column=0,columnspan=4, sticky="nsew")

    button_scale = Button(root, text="Toggle Scale", command=lambda: toggle_scale(scale_var))
    button_scale.grid(row=1, column=0)

    image_subscriber = ImageSubscriber()

    button_service = Button(root, text="Toggle Capture", command=image_subscriber.toggle_capture)
    button_service.grid(row=1, column=1)

    def only_numbers(char):
        return char.isdigit()

    vcmd = (root.register(only_numbers), '%S')

    brightness_var = StringVar(value='0')
    brightness_entry = Entry(root, textvariable=brightness_var, validate='key', validatecommand=vcmd)
    brightness_entry.grid(row=1, column=2)

    button_set_brightness = Button(root, text="Set Brightness", command=lambda: set_brightness(image_subscriber, brightness_var))
    button_set_brightness.grid(row=1, column=3)


    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(image_subscriber)

    def spin():
        while rclpy.ok():
            rclpy.spin_once(image_subscriber, timeout_sec=0.1)

    ros_thread = Thread(target=spin)
    ros_thread.start()

    label.after(100, update_image, label, image_subscriber, scale_var, root)
    root.mainloop()

    executor.shutdown()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
