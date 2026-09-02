/**
 * \file telemetry.hpp
 * \mainpage
 *    Contains the types used to report unsolicited actuator traffic and bus health back to the caller
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__DRIVER__TELEMETRY
#define MYACTUATOR_RMD__DRIVER__TELEMETRY
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>


namespace myactuator_rmd {

  /**\class CachedResponse
   * \brief
   *    A frame that arrived on the bus without having been requested by this driver.
   *
   *    With the active reply feature (command 0xB6) enabled, actuators broadcast their angle, status and torque
   *    at a fixed rate. Those frames share the reply CAN ids and the command bytes of ordinary responses, so a
   *    request-reply driver cannot avoid encountering them. Rather than discarding them, the driver keeps the
   *    most recent one per actuator and per command byte here, so the broadcasts become a free, always-current
   *    state feed instead of interference.
  */
  class CachedResponse {
    public:
      CachedResponse() = default;
      CachedResponse(std::array<std::uint8_t,8> const& data, std::chrono::steady_clock::time_point const& received_at)
      : data_{data}, received_at_{received_at} {
        return;
      }
      CachedResponse(CachedResponse const&) = default;
      CachedResponse& operator = (CachedResponse const&) = default;
      CachedResponse(CachedResponse&&) = default;
      CachedResponse& operator = (CachedResponse&&) = default;

      /**\fn getData
       * \brief
       *    Getter for the eight payload bytes of the frame
       *
       * \return
       *    The payload of the cached frame
      */
      [[nodiscard]]
      constexpr std::array<std::uint8_t,8> const& getData() const noexcept {
        return data_;
      }

      /**\fn getAge
       * \brief
       *    How long ago this frame was taken off the socket
       *
       *    Always check this before acting on a cached value: it tells you whether the actuator is still
       *    broadcasting, or whether you are looking at the last thing it said before it went quiet.
       *
       * \return
       *    The time elapsed since the frame was received
      */
      [[nodiscard]]
      std::chrono::nanoseconds getAge() const noexcept {
        return std::chrono::steady_clock::now() - received_at_;
      }

    protected:
      std::array<std::uint8_t,8> data_ {};
      std::chrono::steady_clock::time_point received_at_ {};
  };

  /**\class BusStatistics
   * \brief
   *    Running counters describing what the driver has seen on the bus.
   *
   *    These exist to make a crowded bus diagnosable from the application: a rising timeout count with a healthy
   *    cache count means replies are being crowded out, whereas a receive buffer far below what was requested
   *    points at net.core.rmem_max rather than at the bus itself.
  */
  class BusStatistics {
    public:
      std::uint64_t num_frames_received {0};  ///< Frames taken off the socket, whatever their origin
      std::uint64_t num_replies_matched {0};  ///< Frames that answered a request this driver had sent
      std::uint64_t num_frames_cached {0};    ///< Frames routed into the unsolicited response cache
      std::uint64_t num_timeouts {0};         ///< Transactions that gave up without seeing their reply
      std::uint64_t num_bus_errors {0};       ///< Non-fatal CAN error frames absorbed rather than thrown
      std::size_t recv_buffer_bytes {0};      ///< Kernel receive buffer the socket was actually granted
      std::string last_bus_error {};          ///< Description of the most recent non-fatal error frame
  };

}

#endif // MYACTUATOR_RMD__DRIVER__TELEMETRY
