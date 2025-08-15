#ifndef ALPHA_5_HARDWARE_INTERFACE_HPP
#define ALPHA_5_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "alpha_5_hardware/alpha_5_driver.h"

namespace alpha_5_hardware{

class Alpha5HardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // SystemInterface overrides
  hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::return_type
    read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type
    write(const rclcpp::Time & time, const rclcpp::Duration & period) override;


private:
  int axis_a_device_id;
  int axis_b_device_id;
  int axis_c_device_id;
  int axis_d_device_id; 
  int axis_e_device_id;
  std::string port;
  struct driver_context ctx;
  
  double prev_axis_a_pos_ = 0.0;
  double prev_axis_b_pos_ = 0.0;
  double prev_axis_c_pos_ = 0.0;
  double prev_axis_d_pos_ = 0.0;
  double prev_axis_e_pos_ = 0.0;

}; //class Alpha5HardwareInterface

} // namespace alpha_5_hardware


#endif