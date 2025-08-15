#include "alpha_5_hardware/alpha_5_hardware_interface.hpp"
#include "alpha_5_hardware/alpha_5_driver.h"
#include "alpha_5_hardware/packetID.h"

namespace alpha_5_hardware{

hardware_interface::CallbackReturn Alpha5HardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
  if (
      hardware_interface::SystemInterface::on_init(info_) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info; // info_ private attribute from SystemInterface

  axis_a_device_id = 0x01;
  axis_b_device_id = 0x02;
  axis_c_device_id = 0x03;
  axis_d_device_id = 0x04;
  axis_e_device_id = 0x05;
  port = info_.hardware_parameters["manipulator_serial_port"];

  return hardware_interface::CallbackReturn::SUCCESS; 
}

hardware_interface::CallbackReturn Alpha5HardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state; // I'm not using previous_state, suppress warning

  // Point to driver initialization method
  ctx = alpha_5_driver_init(port.c_str());
  
  if (ctx.encode_func == NULL) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Alpha5HardwareInterface"),
      "Driver initialization failed for port: %s",
      port.c_str()
    );
    // You can also add errno info if alpha_5_driver_init sets errno:
    RCLCPP_ERROR(
      rclcpp::get_logger("Alpha5HardwareInterface"),
      "errno=%d (%s)",
      errno,
      strerror(errno)
    );
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("Alpha5HardwareInterface"),
    "Driver initialized successfully on port: %s",
    port.c_str()
  );

  return hardware_interface::CallbackReturn::SUCCESS; 
}

hardware_interface::CallbackReturn Alpha5HardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  
  // Initialize the state of the axes
  set_state("axis_a/position", 0.0);
  set_state("axis_a/velocity", 0.0);

  set_state("axis_b/position", 0.0);
  set_state("axis_b/velocity", 0.0);

  set_state("axis_c/position", 0.0);
  set_state("axis_c/velocity", 0.0);

  set_state("axis_d/position", 1.56);
  set_state("axis_d/velocity", 0.0);

  set_state("axis_e/position", 0.0);
  set_state("axis_e/velocity", 0.0);

  // TODO: Point to driver's position mode activation method
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Alpha5HardwareInterface::on_deactivate
  (const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  // TODO: Point to driver's position mode deactivation method
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Alpha5HardwareInterface::read
  (const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Read data from the hardware
  (void)time; // Suppress unused parameter warning

  // TODO: Get position data from the driver

  float axis_a_pos = request(&ctx, axis_a_device_id, POSITION, 50, 2, 3); 
  float axis_b_pos = request(&ctx, axis_b_device_id, POSITION, 50, 2, 3);  
  float axis_c_pos = request(&ctx, axis_c_device_id, POSITION, 50, 2, 3);  
  float axis_d_pos = request(&ctx, axis_d_device_id, POSITION, 50, 2, 3);  
  float axis_e_pos = request(&ctx, axis_e_device_id, POSITION, 50, 2, 3);  

  float dt = period.seconds();

  // Compute velocities
  float axis_a_vel = (axis_a_pos - prev_axis_a_pos_) / dt;
  float axis_b_vel = (axis_b_pos - prev_axis_b_pos_) / dt;
  float axis_c_vel = (axis_c_pos - prev_axis_c_pos_) / dt;
  float axis_d_vel = (axis_d_pos - prev_axis_d_pos_) / dt;
  float axis_e_vel = (axis_e_pos - prev_axis_e_pos_) / dt;
  
  // Update state interfaces 
  set_state("axis_a/position", static_cast<double>(axis_a_pos));
  set_state("axis_a/velocity", static_cast<double>(axis_a_vel));

  set_state("axis_b/position", static_cast<double>(axis_b_pos));
  set_state("axis_b/velocity", static_cast<double>(axis_b_vel));

  set_state("axis_c/position", static_cast<double>(axis_c_pos));
  set_state("axis_c/velocity", static_cast<double>(axis_c_vel));

  set_state("axis_d/position", static_cast<double>(axis_d_pos));
  set_state("axis_d/velocity", static_cast<double>(axis_d_vel));

  set_state("axis_e/position", static_cast<double>(axis_e_pos));
  set_state("axis_e/velocity", static_cast<double>(axis_e_vel));

  prev_axis_a_pos_ = axis_a_pos;
  prev_axis_b_pos_ = axis_b_pos;
  prev_axis_c_pos_ = axis_c_pos;
  prev_axis_d_pos_ = axis_d_pos;
  prev_axis_e_pos_ = axis_e_pos;

  RCLCPP_INFO(get_logger(), " POSITION STATE axis_a: %lf, axis_b: %lf, axis_c: %lf, axis_d: %lf, axis_e: %lf", 
  get_state("axis_a/position"), get_state("axis_b/position"), get_state("axis_c/position"), 
  get_state("axis_d/position"), get_state("axis_e/position"));

  RCLCPP_INFO(get_logger(), " VELOCITY STATE axis_a: %lf, axis_b: %lf, axis_c: %lf, axis_d: %lf, axis_e: %lf", 
  get_state("axis_a/velocity"), get_state("axis_b/velocity"), get_state("axis_c/velocity"), 
  get_state("axis_d/velocity"), get_state("axis_e/velocity"));

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Alpha5HardwareInterface::write
  (const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  // get_command("axis_/position") from the command interfaces and send position data to the driver
  sendPosition(&ctx, axis_a_device_id, get_command("axis_a/position"), 50);
  sendPosition(&ctx, axis_b_device_id, get_command("axis_b/position"), 50);
  sendPosition(&ctx, axis_c_device_id, get_command("axis_c/position"), 50);
  sendPosition(&ctx, axis_d_device_id, get_command("axis_d/position"), 50);
  sendPosition(&ctx, axis_e_device_id, get_command("axis_e/position"), 50);

  RCLCPP_INFO(get_logger(), " COMMAND STATE axis_a: %lf, axis_b: %lf, axis_c: %lf, axis_d: %lf, axis_e: %lf", 
  get_command("axis_a/position"), get_command("axis_b/position"), get_command("axis_c/position"), 
  get_command("axis_d/position"), get_command("axis_e/position"));

  return hardware_interface::return_type::OK;
} 

} // namespace alpha_5_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(alpha_5_hardware::Alpha5HardwareInterface, hardware_interface::SystemInterface)