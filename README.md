based on : https://github.com/ros-drivers/usb_cam, and modified by https://github.com/boitumeloruf/usb_cam


# run using launchfile:
example:
  ros2 launch usb_cam smartFishing_launch.py


# get available parameters:
  ros2 param list /usb_cam


# chage availabe parameters:
  ros2 param set /usb_cam white_balance_automatic true
