// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "usb_cam/usb_cam_node.hpp"
#include "usb_cam/utils.hpp"

const char BASE_TOPIC_NAME[] = "image_raw";

namespace usb_cam
{

  UsbCamNode::UsbCamNode(const rclcpp::NodeOptions &node_options)
      : Node("usb_cam", node_options),
        // m_camera(new usb_cam::UsbCam()),
        m_camera(new usb_cam::UsbCam([this](size_t size) -> void
                                     { return this->resize_ros_image_buffer_if_nessesary(size); })),
        m_image_msg(new sensor_msgs::msg::Image()),
        m_compressed_img_msg(nullptr),
        m_image_publisher(std::make_shared<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(this, BASE_TOPIC_NAME,
                                                     rclcpp::QoS{100}.get_rmw_qos_profile()))),
        m_compressed_image_publisher(nullptr),
        m_compressed_cam_info_publisher(nullptr),
        m_parameters(),
        m_camera_info_msg(new sensor_msgs::msg::CameraInfo()),
        m_service_capture(
            this->create_service<std_srvs::srv::SetBool>(
                "set_capture",
                std::bind(
                    &UsbCamNode::service_capture,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3)))
  {
    this->declare_parameter("video_device", "/dev/video0");
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

    std::string device_path = this->get_parameter("video_device").as_string();
    RCLCPP_INFO(this->get_logger(), "device_path value: %s", device_path.c_str());
    m_parameters.device_name = device_path;
    cam_params = get_control_data(device_path);
    // Get camera parameters using V4L2

    // declare params
    // get full ist of supported parameters by calling 'v4l2-ctl --device=<device> -L'
    this->declare_parameter("camera_name", "default_cam");
    this->declare_parameter("camera_info_url", "");
    this->declare_parameter("framerate", 30.0);
    this->declare_parameter("frame_id", "default_cam");
    this->declare_parameter("image_height", 480);
    this->declare_parameter("image_width", 640);
    this->declare_parameter("io_method", "mmap");
    this->declare_parameter("pixel_format", "yuyv");
    this->declare_parameter("av_device_format", "YUV422P");
    declare_camera_parameters(this, cam_params);

    assign_params();
    init();
    m_parameters_callback_handle = add_on_set_parameters_callback(
        std::bind(
            &UsbCamNode::parameters_callback, this,
            std::placeholders::_1));
  }
  void UsbCamNode::resize_ros_image_buffer_if_nessesary(size_t size)
  {
    // Only resize if required
    if (sizeof(m_compressed_img_msg->data) != size)
    {
      m_compressed_img_msg->data.resize(size);
    }
  }

  UsbCamNode::~UsbCamNode()
  {
    RCLCPP_WARN(this->get_logger(), "Shutting down");
    m_image_msg.reset();
    m_compressed_img_msg.reset();
    m_camera_info_msg.reset();
    m_camera_info.reset();
    m_timer.reset();
    m_service_capture.reset();
    m_parameters_callback_handle.reset();

    delete (m_camera);
  }

std::string UsbCamNode::control_type_to_string(__u32 type)
{
  switch (type)
  {
  case V4L2_CTRL_TYPE_INTEGER:
    return "int";
  case V4L2_CTRL_TYPE_BOOLEAN:
    return "bool";
  case V4L2_CTRL_TYPE_MENU:
    return "int";
  case V4L2_CTRL_TYPE_BUTTON:
    return "button";
  case V4L2_CTRL_TYPE_INTEGER64:
    return "int64_t";
  case V4L2_CTRL_TYPE_CTRL_CLASS:
    return "class";
  case V4L2_CTRL_TYPE_STRING:
    return "string";
  case V4L2_CTRL_TYPE_BITMASK:
    return "bitmask";
  case V4L2_CTRL_TYPE_INTEGER_MENU:
    return "int_menu";
  default:
    return "unknown";
  }
}

  void UsbCamNode::declare_camera_parameters(rclcpp::Node *node, std::vector<ControlInfo> cam_params)
  {
    // Iterate through each parameter and declare it in the node
    for (const auto &param : cam_params)
    {
      std::string control_type = control_type_to_string(param.type);
      if (control_type == "int" || control_type == "int64_t" || control_type == "int_menu")
      {
        RCLCPP_INFO(node->get_logger(), "int %s", param.name.c_str());
        node->declare_parameter(param.name, param.default_value);
      }
      else if (control_type == "bool")
      {
        RCLCPP_INFO(node->get_logger(), "bool %s", param.name.c_str());

        bool default_bool_value = param.default_value != 0; // Convert to boolean
        node->declare_parameter(param.name, default_bool_value);
      }
      else if (control_type == "string")
      {
        // Declare a string parameter with an empty default value or based on control defaults if applicable
        node->declare_parameter(param.name, std::string(""));
      }
      else if (control_type == "bitmask")
      {
        node->declare_parameter(param.name, param.default_value); // Treat bitmask as an integer
      }
      else if (control_type == "menu")
      {
        // Declare an integer for the menu type (assuming default_value is valid)
        node->declare_parameter(param.name, param.default_value);
      }
      else if (control_type == "button" || control_type == "class" || control_type == "unknown")
      {
        // For types like button or unknown, we might not want to declare them as parameters
        RCLCPP_INFO(node->get_logger(), "Skipping unsupported parameter type: %s", control_type.c_str());
      }
      // Print out the parameter name for debugging
      RCLCPP_INFO_STREAM(node->get_logger(), "Declared parameter: " << param.name << "; control_type:" << control_type);
    }
  }


  std::vector<ControlInfo> UsbCamNode::get_control_data(const std::string &device)
  {
    std::vector<ControlInfo> controls;
    int fd = open(device.c_str(), O_RDWR);
    if (fd == -1)
    {
      perror("Opening video device");
      return controls;
    }

    struct v4l2_queryctrl queryctrl;
    struct v4l2_querymenu querymenu;
    memset(&queryctrl, 0, sizeof(queryctrl));

    queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    while (0 == ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl))
    {
      // Filter out disabled controls and any top-level control categories
      if (!(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) &&
          std::strcmp(reinterpret_cast<char *>(queryctrl.name), "User Controls") != 0 &&
          std::strcmp(reinterpret_cast<char *>(queryctrl.name), "Camera Controls") != 0)
      {
        ControlInfo info;

        // Convert control name to string and format it if necessary
        std::string control_name = reinterpret_cast<char *>(queryctrl.name);
        control_name.erase(std::remove(control_name.begin(), control_name.end(), ','), control_name.end()); // Remove commas
        std::replace(control_name.begin(), control_name.end(), ' ', '_');                                   // Replace spaces with underscores
        std::transform(control_name.begin(), control_name.end(), control_name.begin(), [](unsigned char c)
                       { return std::tolower(c); });

        info.name = control_name;
        info.type = queryctrl.type;
        info.minimum = queryctrl.minimum;
        info.maximum = queryctrl.maximum;
        info.step = queryctrl.step;
        info.default_value = queryctrl.default_value;

        // If the control type is a menu, populate the menu items
        if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
        {
          memset(&querymenu, 0, sizeof(querymenu));
          querymenu.id = queryctrl.id;

          for (querymenu.index = queryctrl.minimum; querymenu.index <= static_cast<unsigned int>(queryctrl.maximum); querymenu.index++)
          {
            if (0 == ioctl(fd, VIDIOC_QUERYMENU, &querymenu))
            {
              std::string menu_item_name = std::to_string(querymenu.index) + ": " + reinterpret_cast<char *>(querymenu.name);
              menu_item_name.erase(std::remove(menu_item_name.begin(), menu_item_name.end(), ','), menu_item_name.end()); // Remove commas
              info.menu_items.push_back(menu_item_name);
            }
          }
        }

        // Print control info for debugging
        std::cout << "Control Name: " << info.name << std::endl;
        std::cout << "Type: " << info.type << std::endl;
        std::cout << "Min: " << info.minimum << ", Max: " << info.maximum << ", Step: " << info.step << ", Default: " << info.default_value << std::endl;

        // Add control info to the list
        controls.push_back(info);
      }
      // Move to the next control
      queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }

    if (errno != EINVAL)
    {
      perror("VIDIOC_QUERYCTRL");
    }

    close(fd);
    return controls;
  }

  void UsbCamNode::service_capture(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    (void)request_header;
    if (request->data)
    {
      m_camera->start_capturing();
      response->message = "Start Capturing";
    }
    else
    {
      m_camera->stop_capturing();
      response->message = "Stop Capturing";
    }
  }

  void UsbCamNode::init()
  {
    while (m_parameters.frame_id == "")
    {
      RCLCPP_WARN_ONCE(
          this->get_logger(), "Required Parameters not set...waiting until they are set");
      get_params();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // load the camera info
    m_camera_info.reset(
        new camera_info_manager::CameraInfoManager(
            this, m_parameters.camera_name, m_parameters.camera_info_url));
    // check for default camera info
    if (!m_camera_info->isCalibrated())
    {
      m_camera_info->setCameraName(m_parameters.device_name);
      m_camera_info_msg->header.frame_id = m_parameters.frame_id;
      m_camera_info_msg->width = m_parameters.image_width;
      m_camera_info_msg->height = m_parameters.image_height;
      m_camera_info->setCameraInfo(*m_camera_info_msg);
    }

    // Check if given device name is an available v4l2 device
    auto available_devices = usb_cam::utils::available_devices();
    if (available_devices.find(m_parameters.device_name) == available_devices.end())
    {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Device specified is not available or is not a vaild V4L2 device: `" << m_parameters.device_name << "`");
      RCLCPP_INFO(this->get_logger(), "Available V4L2 devices are:");
      for (const auto &device : available_devices)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "    " << device.first);
        RCLCPP_INFO_STREAM(this->get_logger(), "        " << device.second.card);
      }
      rclcpp::shutdown();
      return;
    }

    // if pixel format is equal to 'mjpeg', i.e. raw mjpeg stream, initialize compressed image message
    // and publisher
    if (m_parameters.pixel_format == "mjpeg")
    {
      m_compressed_img_msg.reset(new sensor_msgs::msg::CompressedImage());
      m_compressed_img_msg->header.frame_id = m_parameters.frame_id;
      m_compressed_image_publisher =
          this->create_publisher<sensor_msgs::msg::CompressedImage>(
              std::string(BASE_TOPIC_NAME) + "/compressed", rclcpp::QoS(100));
      m_compressed_cam_info_publisher =
          this->create_publisher<sensor_msgs::msg::CameraInfo>(
              "camera_info", rclcpp::QoS(100));
    }

    m_image_msg->header.frame_id = m_parameters.frame_id;
    RCLCPP_INFO(
        this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
        m_parameters.camera_name.c_str(), m_parameters.device_name.c_str(),
        m_parameters.image_width, m_parameters.image_height, m_parameters.io_method.c_str(),
        m_parameters.pixel_format.c_str(), m_parameters.framerate);
    // set the IO method
    io_method_t io_method =
        usb_cam::utils::io_method_from_string(m_parameters.io_method);
    if (io_method == usb_cam::utils::IO_METHOD_UNKNOWN)
    {
      RCLCPP_ERROR_ONCE(
          this->get_logger(),
          "Unknown IO method '%s'", m_parameters.io_method.c_str());
      rclcpp::shutdown();
      return;
    }
    // configure the camera
    m_camera->configure(m_parameters, io_method);

    RCLCPP_INFO(this->get_logger(), "This devices supproted formats:");
    for (auto fmt : m_camera->supported_formats())
    {
      RCLCPP_INFO(
          this->get_logger(),
          "\t%s: %d x %d (%d Hz)",
          fmt.format.description,
          fmt.v4l2_fmt.width,
          fmt.v4l2_fmt.height,
          fmt.v4l2_fmt.discrete.denominator / fmt.v4l2_fmt.discrete.numerator);
    }

    set_v4l2_params();

    // start the camera
    m_camera->start();

    // TODO(lucasw) should this check a little faster than expected frame rate?
    // TODO(lucasw) how to do small than ms, or fractional ms- std::chrono::nanoseconds?
    const int period_ms = 1000.0 / m_parameters.framerate;
    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&UsbCamNode::update, this));
    RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");
  }

  void UsbCamNode::assign_params()
  {
    std::string camera_name = this->get_parameter("camera_name").as_string();
    RCLCPP_INFO(this->get_logger(), "camera_name value: %s", camera_name.c_str());
    m_parameters.camera_name = camera_name;

    std::string camera_info_url = this->get_parameter("camera_info_url").as_string();
    RCLCPP_INFO(this->get_logger(), "camera_info_url value: %s", camera_info_url.c_str());
    m_parameters.camera_info_url = camera_info_url;

    std::string frame_id = this->get_parameter("frame_id").as_string();
    RCLCPP_INFO(this->get_logger(), "frame_id value: %s", frame_id.c_str());
    m_parameters.frame_id = frame_id;

    double framerate = this->get_parameter("framerate").as_double();
    RCLCPP_INFO(this->get_logger(), "framerate value: %f", framerate);
    m_parameters.framerate = framerate;

    int image_height = this->get_parameter("image_height").as_int();
    RCLCPP_INFO(this->get_logger(), "image_height value: %d", image_height);
    m_parameters.image_height = image_height;

    int image_width = this->get_parameter("image_width").as_int();
    RCLCPP_INFO(this->get_logger(), "image_width value: %d", image_width);
    m_parameters.image_width = image_width;

    std::string io_method = this->get_parameter("io_method").as_string();
    RCLCPP_INFO(this->get_logger(), "io_method value: %s", io_method.c_str());
    m_parameters.io_method = io_method;

    std::string pixel_format = this->get_parameter("pixel_format").as_string();
    RCLCPP_INFO(this->get_logger(), "pixel_format value: %s", pixel_format.c_str());
    m_parameters.pixel_format = pixel_format;

    std::string av_device_format = this->get_parameter("av_device_format").as_string();
    RCLCPP_INFO(this->get_logger(), "av_device_format value: %s", av_device_format.c_str());
    m_parameters.av_device_format = av_device_format;

    // Iterate through each parameter and declare it in the node
    for (auto &param : cam_params)
    {
      std::string control_type = control_type_to_string(param.type);
      if (control_type == "int" || control_type == "int64_t" || control_type == "int_menu")
      {
        param.set_value = this->get_parameter(param.name).as_int();
        RCLCPP_INFO(this->get_logger(), "%s value: %d", param.name.c_str(), param.set_value);
      }
      else if (control_type == "bool")
      {
        param.set_value = this->get_parameter(param.name).as_bool();
        RCLCPP_INFO(this->get_logger(), "%s value: %d", param.name.c_str(), param.set_value);
      }
    }
  }

  /// @brief Send current parameters to V4L2 device
  /// TODO(flynneva): should this actuaully be part of UsbCam class?
  void UsbCamNode::set_v4l2_params()
  {
    // Iterate through each parameter and declare it in the node
    for (const auto &param : cam_params)
    {
      // std::string control_type = control_type_to_string(param.type);
      // if (control_type == "int" || control_type == "int64_t" || control_type == "int_menu")
      {
        if (param.set_value != -1 && param.default_value != param.set_value)
        {
          RCLCPP_INFO(this->get_logger(), "Setting '%s' to %d", param.name.c_str(), param.default_value);
          m_camera->set_v4l_parameter(param.name, param.set_value);
        }
      }
      // else if (control_type == "bool")
      // {
      //   RCLCPP_INFO(this->get_logger(), "Setting '%s' to %d", param.name.c_str(), param.default_value);
      //   m_camera->set_v4l_parameter(param.name, param.set_value);
      // }
    }
  }

  bool UsbCamNode::take_and_send_image()
  {
    // Only resize if required
    if (sizeof(m_image_msg->data) != m_camera->get_image_size_in_bytes())
    {
      m_image_msg->width = m_camera->get_image_width();
      m_image_msg->height = m_camera->get_image_height();
      m_image_msg->encoding = m_camera->get_pixel_format()->ros();
      m_image_msg->step = m_camera->get_image_step();
      if (m_image_msg->step == 0)
      {
        // Some formats don't have a linesize specified by v4l2
        // Fall back to manually calculating it step = size / height
        m_image_msg->step = m_camera->get_image_size_in_bytes() / m_image_msg->height;
      }
      m_image_msg->data.resize(m_camera->get_image_size_in_bytes());
    }

    // grab the image, pass image msg buffer to fill
    m_camera->get_image(reinterpret_cast<char *>(&m_image_msg->data[0]));

    auto stamp = m_camera->get_image_timestamp();
    m_image_msg->header.stamp.sec = stamp.tv_sec;
    m_image_msg->header.stamp.nanosec = stamp.tv_nsec;

    *m_camera_info_msg = m_camera_info->getCameraInfo();
    m_camera_info_msg->header = m_image_msg->header;
    m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);
    return true;
  }

  bool UsbCamNode::take_and_send_image_mjpeg()
  {
    // Only resize if required
    if (sizeof(m_compressed_img_msg->data) != m_camera->get_image_size_in_bytes())
    {
      m_compressed_img_msg->format = "jpeg";
      m_compressed_img_msg->data.resize(m_camera->get_image_size_in_bytes());
    }

    // grab the image, pass image msg buffer to fill
    m_camera->get_image(reinterpret_cast<char *>(&m_compressed_img_msg->data[0]));

    auto stamp = m_camera->get_image_timestamp();
    m_compressed_img_msg->header.stamp.sec = stamp.tv_sec;
    m_compressed_img_msg->header.stamp.nanosec = stamp.tv_nsec;

    *m_camera_info_msg = m_camera_info->getCameraInfo();
    m_camera_info_msg->header = m_compressed_img_msg->header;

    m_compressed_image_publisher->publish(*m_compressed_img_msg);
    m_compressed_cam_info_publisher->publish(*m_camera_info_msg);
    return true;
  }

  rcl_interfaces::msg::SetParametersResult UsbCamNode::parameters_callback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    RCLCPP_DEBUG(this->get_logger(), "Setting parameters for %s", m_parameters.camera_name.c_str());
    m_timer->reset();
    assign_params();
    RCLCPP_DEBUG(this->get_logger(), " %ld", parameters.size());

    set_v4l2_params();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  void UsbCamNode::update()
  {
    if (m_camera->is_capturing())
    {
      // If the camera exposure longer higher than the framerate period
      // then that caps the framerate.
      // auto t0 = now();
      bool isSuccessful = (m_parameters.pixel_format == "mjpeg") ? take_and_send_image_mjpeg() : take_and_send_image();
      if (!isSuccessful)
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "USB camera did not respond in time.");
      }
    }
  }
} // namespace usb_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::UsbCamNode)
