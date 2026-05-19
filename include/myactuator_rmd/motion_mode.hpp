#ifndef MYACTUATOR_RMD__MOTION_MODE
#define MYACTUATOR_RMD__MOTION_MODE
#pragma once

#include "myactuator_rmd/driver/can_node.hpp"
#include "myactuator_rmd/protocol/motion_mode_message.hpp"

namespace myactuator_rmd {
namespace motion_mode {

  // Uses 0x400 to Send and 0x500 to Receive!
  class CanDriver : public CanNode<0x400, 0x500> {
    public:
      CanDriver(std::string const& ifname) : CanNode<0x400, 0x500>{ifname} {}
  };

  class ActuatorInterface {
    public:
      ActuatorInterface(CanDriver& driver, std::uint32_t actuator_id)
      : driver_{driver}, actuator_id_{actuator_id} {
          driver_.addId(actuator_id_);
      }

      MotionModeResponse sendMotionModeSetpoint(float p_des, float v_des, float kp, float kd, float t_ff) {
          MotionModeRequest const request{p_des, v_des, kp, kd, t_ff};
          auto const response_data = driver_.sendRecv(request, actuator_id_);
          return MotionModeResponse{response_data};
      }

    private:
      CanDriver& driver_;
      std::uint32_t actuator_id_;
  };

}
}
#endif