/**
 * \file driver.hpp
 * \mainpage
 *    Contains the driver base class defining the interface
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__DRIVER__DRIVER
#define MYACTUATOR_RMD__DRIVER__DRIVER
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "myactuator_rmd/driver/telemetry.hpp"
#include "myactuator_rmd/protocol/message.hpp"


namespace myactuator_rmd {

  class ActuatorInterface;

  /**\class Driver
   * \brief
   *    Pure abstract base class for drivers
  */
  class Driver {
    public:
      virtual ~Driver() = default;

      /**\fn addId
       * \brief
       *    Updates the id as well as the send and receive ids in a consistent manner
       *
       * \param[in] actuator_id
       *    The id of the actuator
      */
      virtual void addId(std::uint32_t const actuator_id) = 0;

      /**\fn send
       * \brief
       *    Writes the given data to the participant with the actuator id actuator_id
       *
       * \param[in] msg
       *    The message that should be sent to the corresponding actuator
       * \param[in] actuator_id
       *    The ID of the actuator that the message should be sent to
      */
      virtual void send(Message const& msg, std::uint32_t const actuator_id) = 0;

      /**\fn sendRecv
       * \brief
       *    Writes the given data to the actuator with the corresponding id and waits for a corresponding reply
       *
       * \param[in] request
       *    Request that should be sent to the corresponding actuator
       * \param[in] actuator_id
       *    The ID of the actuator that the message should be sent to
       * \return
       *    The response bytes
      */
      [[nodiscard]]
      virtual std::array<std::uint8_t,8> sendRecv(Message const& request, std::uint32_t const actuator_id) = 0;

      /**\fn setResponseTimeout
       * \brief
       *    Set how long a single request-reply transaction may wait for its own reply before giving up
       *
       *    This is deliberately separate from the socket receive timeout: it bounds the whole transaction rather
       *    than each individual frame, so a reply that is queued behind unrelated broadcast traffic does not get
       *    a fresh timeout for every frame that has to be skipped over first.
       *
       * \param[in] timeout
       *    The maximum duration a transaction may wait for its reply
      */
      virtual void setResponseTimeout(std::chrono::microseconds const& timeout) {
        static_cast<void>(timeout);
        return;
      }

      /**\fn getResponseTimeout
       * \brief
       *    Get the current per-transaction reply timeout
       *
       * \return
       *    The maximum duration a transaction may wait for its reply
      */
      [[nodiscard]]
      virtual std::chrono::microseconds getResponseTimeout() const {
        return std::chrono::microseconds(0);
      }

      /**\fn getCachedResponse
       * \brief
       *    Get the most recent unsolicited frame received from an actuator for a given command byte
       *
       *    This is how active-reply broadcasts (command 0xB6) are consumed: the driver parks every frame that
       *    was not the answer to a request here, so the broadcasts can be read back without putting a single
       *    additional request on the bus.
       *
       * \param[in] actuator_id
       *    The ID of the actuator the frame should have come from
       * \param[in] command
       *    The command byte the frame was tagged with, e.g. 0x92 for the multi-turn angle
       * \return
       *    The cached frame, or no value if nothing has been received for that actuator and command
      */
      [[nodiscard]]
      virtual std::optional<CachedResponse> getCachedResponse(std::uint32_t const actuator_id, std::uint8_t const command) const {
        static_cast<void>(actuator_id);
        static_cast<void>(command);
        return std::nullopt;
      }

      /**\fn pollTelemetry
       * \brief
       *    Move everything currently queued on the socket into the unsolicited response cache without sending
       *    anything
       *
       *    Transactions do this implicitly, so an application that talks to its actuators continuously never
       *    needs to call it. It is there for the gaps: call it from an idle loop so that broadcasts arriving
       *    while nothing is being commanded are still collected rather than accumulating in the socket buffer.
       *
       * \return
       *    The number of frames that were moved into the cache
      */
      virtual std::size_t pollTelemetry() {
        return 0;
      }

      /**\fn getStatistics
       * \brief
       *    Get the running bus counters, see BusStatistics
       *
       * \return
       *    The counters accumulated since the driver was constructed
      */
      [[nodiscard]]
      virtual BusStatistics getStatistics() const {
        return BusStatistics{};
      }

      /**\fn resetStatistics
       * \brief
       *    Zero the running bus counters, so a subsequent measurement covers a known window
      */
      virtual void resetStatistics() {
        return;
      }

    protected:
      Driver() = default;
      Driver(Driver const&) = default;
      Driver& operator = (Driver const&) = default;
      Driver(Driver&&) = default;
      Driver& operator = (Driver&&) = default;

      friend ActuatorInterface;
  };

}

#endif // MYACTUATOR_RMD__DRIVER__DRIVER
