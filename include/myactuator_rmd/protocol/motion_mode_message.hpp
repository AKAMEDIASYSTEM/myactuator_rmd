#ifndef MYACTUATOR_RMD__PROTOCOL__MOTION_MODE_MESSAGE
#define MYACTUATOR_RMD__PROTOCOL__MOTION_MODE_MESSAGE
#pragma once

#include "myactuator_rmd/protocol/message.hpp"
#include <algorithm>

namespace myactuator_rmd {
namespace motion_mode {

  // Helper to map float ranges into integers for CAN transmission
  inline std::uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
      float span = x_max - x_min;
      float offset = x_min;
      x = std::clamp(x, x_min, x_max);
      return static_cast<std::uint16_t>(((x - offset) * static_cast<float>((1 << bits) - 1)) / span);
  }

  inline float uint_to_float(std::uint16_t x, float x_min, float x_max, int bits) {
      float span = x_max - x_min;
      float offset = x_min;
      return ((static_cast<float>(x) * span) / static_cast<float>((1 << bits) - 1)) + offset;
  }

  class MotionModeRequest : public Message {
    public:
      // Removed t_max. t_ff_percent now explicitly expects -100.0 to 100.0
      MotionModeRequest(float p_des, float v_des, float kp, float kd, float t_ff_percent, float kp_max = 500.0f, float kd_max = 5.0f) {
          float const P_MIN = -12.566f, P_MAX = 12.566f;
          float const V_MIN = -45.0f,   V_MAX = 45.0f;
          float const KP_MIN = 0.0f;
          float const KD_MIN = 0.0f;
          float const T_MIN = -100.0f,  T_MAX = 100.0f; // -100% to 100% of Max Torque

          std::uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
          std::uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
          std::uint16_t kp_int = float_to_uint(kp, KP_MIN, kp_max, 12);
          std::uint16_t kd_int = float_to_uint(kd, KD_MIN, kd_max, 12);
          std::uint16_t t_int = float_to_uint(t_ff_percent, T_MIN, T_MAX, 12);

          data_[0] = p_int >> 8;
          data_[1] = p_int & 0xFF;
          data_[2] = v_int >> 4;
          data_[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
          data_[4] = kp_int & 0xFF;
          data_[5] = kd_int >> 4;
          data_[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
          data_[7] = t_int & 0xFF;
      }
  };

  class MotionModeResponse : public Message {
    public:
      MotionModeResponse(std::array<std::uint8_t, 8> const& data) : Message(data) {}

      float getPosition() const {
          std::uint16_t p_int = (data_[1] << 8) | data_[2];
          return uint_to_float(p_int, -12.566f, 12.566f, 16);
      }

      float getVelocity() const {
          std::uint16_t v_int = (data_[3] << 4) | (data_[4] >> 4);
          return uint_to_float(v_int, -45.0f, 45.0f, 12);
      }

      // Renamed to clarify it returns a percentage from -100.0 to 100.0
      float getTorquePercent() const {
          std::uint16_t t_int = ((data_[4] & 0xF) << 8) | data_[5];
          return uint_to_float(t_int, -100.0f, 100.0f, 12);
      }
  };

}
}
#endif