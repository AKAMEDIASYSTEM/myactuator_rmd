/**
 * \file can_node.hpp
 * \mainpage
 *    Contains the base class for the CAN driver as well as the actuator mock
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__DRIVER__CAN_NODE
#define MYACTUATOR_RMD__DRIVER__CAN_NODE
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ios>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "myactuator_rmd/can/exceptions.hpp"
#include "myactuator_rmd/can/frame.hpp"
#include "myactuator_rmd/can/node.hpp"
#include "myactuator_rmd/driver/driver.hpp"
#include "myactuator_rmd/driver/telemetry.hpp"
#include "myactuator_rmd/protocol/message.hpp"
#include "myactuator_rmd/exceptions.hpp"


namespace myactuator_rmd {

  /**\class ResponseMatching
   * \brief
   *    Describes how a reply is recognised as belonging to a particular request on a given CAN id range.
   *
   *    On the standard 0x140/0x240 range every reply repeats the request's command byte in data[0], so a reply
   *    is identified by the pair (reply CAN id, command byte). That second half matters: with active replies
   *    enabled an actuator emits 0x92/0x9A/0x9C frames on its own reply id at a fixed rate, so the CAN id alone
   *    does not distinguish the answer to a position command from a broadcast that happened to arrive first.
   *
   * \tparam SEND_ID_OFFSET
   *    The CAN id offset that requests are sent to, which identifies the protocol variant in use
  */
  template <std::uint32_t SEND_ID_OFFSET>
  class ResponseMatching {
    public:
      inline static constexpr bool has_command_echo {true};
  };

  /**\class ResponseMatching<0x400>
   * \brief
   *    The motion mode (MIT-style) range at 0x400/0x500 packs position data into data[0] instead of a command
   *    identifier, so there is no command byte to compare and replies are matched on their CAN id alone. That is
   *    sufficient there because motion mode replies arrive on 0x500 + id, which the active reply broadcasts on
   *    0x240 + id never collide with.
  */
  template <>
  class ResponseMatching<0x400> {
    public:
      inline static constexpr bool has_command_echo {false};
  };

  /**\class CanNode
   * \brief
   *    Base class for the CAN driver as well as the actuator mock
  */
  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  class CanNode: public Driver, protected can::Node {
    public:
      /**\var default_response_timeout
       * \brief
       *    How long a transaction waits for its reply unless told otherwise.
       *
       *    An RMD actuator answers within well under a millisecond, so this is generous. It is nonetheless far
       *    shorter than the one second socket receive timeout it replaces, because on a bus with twenty
       *    actuators a per-transaction stall of a second makes a control loop unusable, and a reply that has
       *    not arrived in a hundred milliseconds is not going to arrive at all.
      */
      inline static constexpr std::chrono::microseconds default_response_timeout {std::chrono::milliseconds(100)};

      /**\var default_recv_buffer_bytes
       * \brief
       *    Receive buffer requested for the socket, roughly two orders of magnitude above the usual default.
       *
       *    Twenty actuators broadcasting three active replies each at 10 Hz put 600 frames a second on the
       *    socket. The stock buffer holds well under a second of that, so any pause in the application costs
       *    silently dropped frames. This asks for enough headroom to absorb several seconds of it.
      */
      inline static constexpr std::size_t default_recv_buffer_bytes {4*1024*1024};

      void setResponseTimeout(std::chrono::microseconds const& timeout) override;

      [[nodiscard]]
      std::chrono::microseconds getResponseTimeout() const override;

      [[nodiscard]]
      std::optional<CachedResponse> getCachedResponse(std::uint32_t const actuator_id, std::uint8_t const command) const override;

      std::size_t pollTelemetry() override;

      [[nodiscard]]
      BusStatistics getStatistics() const override;

      void resetStatistics() override;

    protected:
      /**\fn CanNode
       * \brief
       *    Class constructor
       *
       * \param[in] ifname
       *    The name of the network interface that should communicated over
      */
      CanNode(std::string const& ifname);
      CanNode() = delete;
      CanNode(CanNode const&) = delete;
      CanNode& operator = (CanNode const&) = delete;
      CanNode(CanNode&&) = delete;
      CanNode& operator = (CanNode&&) = delete;

      /**\fn addId
       * \brief
       *    Updates the id as well as the send and receive ids in a consistent manner
       *
       * \param[in] actuator_id
       *    The id of the actuator [0, 32]
      */
      void addId(std::uint32_t const actuator_id) override;

      /**\fn send
       * \brief
       *    Writes a given CAN frame based on the request to the CAN participant with the
       *    actuator id actuator_id
       *
       * \param[in] msg
       *    The message that should be sent to the corresponding actuator
       * \param[in] actuator_id
       *    The ID of the actuator that the message should be sent to
      */
      void send(Message const& msg, std::uint32_t const actuator_id) override;

      /**\fn sendRecv
       * \brief
       *    Writes a given CAN frame based on the request to the actuator with the corresponding id
       *    and waits for the reply that actually belongs to that request
       *
       * \param[in] request
       *    Request that should be sent to the corresponding actuator
       * \param[in] actuator_id
       *    The ID of the actuator that the message should be sent to
       * \return
       *    The response bytes
      */
      [[nodiscard]]
      std::array<std::uint8_t,8> sendRecv(Message const& request, std::uint32_t const actuator_id) override;

      /**\fn getCanSendId
       * \brief
       *    Get the CAN id that a message should be sent to
       *
       * \param[in] actuator_id
       *    The ID of the actuator that the message should be sent to
       * \return
       *    The CAN id that the message should be sent to
      */
      [[nodiscard]]
      constexpr std::uint32_t getCanSendId(std::uint32_t const actuator_id) const noexcept;

      /**\fn getCanReceiveId
       * \brief
       *    Get the CAN id that a message will be received from
       *
       * \param[in] actuator_id
       *    The ID of the actuator that the message should be received from
       * \return
       *    The CAN id that the message should be received from
      */
      [[nodiscard]]
      constexpr std::uint32_t getCanReceiveId(std::uint32_t const actuator_id) const noexcept;

      /**\fn isReplyTo
       * \brief
       *    Decide whether a received frame is the reply to the request that is currently outstanding
       *
       * \param[in] frame
       *    The frame that was received
       * \param[in] expected_receive_id
       *    The CAN id the reply is expected to arrive on
       * \param[in] expected_command
       *    The command byte the reply is expected to repeat, ignored on ranges without a command echo
       * \return
       *    True if the frame answers the outstanding request
      */
      [[nodiscard]]
      static bool isReplyTo(can::Frame const& frame, std::uint32_t const expected_receive_id,
                            std::uint8_t const expected_command) noexcept;

      /**\fn cacheFrame
       * \brief
       *    Store a frame that was not the reply we were waiting for, keyed by its CAN id and command byte
       *    The caller must hold bus_mutex_
       *
       * \param[in] frame
       *    The frame to store
      */
      void cacheFrame(can::Frame const& frame);

      /**\fn drainPending
       * \brief
       *    Move every frame currently queued on the socket into the cache, without waiting for more to arrive
       *    The caller must hold bus_mutex_
       *
       * \return
       *    The number of frames that were moved
      */
      std::size_t drainPending();

      mutable std::mutex bus_mutex_;
      std::chrono::microseconds response_timeout_;
      // Cached unsolicited frames, indexed first by the CAN id they arrived on and then by their command byte
      std::map<std::uint32_t,std::map<std::uint8_t,CachedResponse>> telemetry_;
      BusStatistics statistics_;
      std::vector<std::uint32_t> actuator_ids_;
  };

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::CanNode(std::string const& ifname)
  : Driver{}, can::Node{ifname}, bus_mutex_{}, response_timeout_{default_response_timeout}, telemetry_{},
    statistics_{}, actuator_ids_{} {
    statistics_.recv_buffer_bytes = can::Node::setRecvBufferSize(default_recv_buffer_bytes);
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  void CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::setResponseTimeout(std::chrono::microseconds const& timeout) {
    std::lock_guard<std::mutex> const lock {bus_mutex_};
    response_timeout_ = timeout;
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  std::chrono::microseconds CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::getResponseTimeout() const {
    std::lock_guard<std::mutex> const lock {bus_mutex_};
    return response_timeout_;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  std::optional<CachedResponse> CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::getCachedResponse(std::uint32_t const actuator_id,
                                                                                             std::uint8_t const command) const {
    std::lock_guard<std::mutex> const lock {bus_mutex_};
    auto const per_actuator {telemetry_.find(getCanReceiveId(actuator_id))};
    if (per_actuator == telemetry_.end()) {
      return std::nullopt;
    }
    auto const entry {per_actuator->second.find(command)};
    if (entry == per_actuator->second.end()) {
      return std::nullopt;
    }
    return entry->second;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  std::size_t CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::pollTelemetry() {
    std::lock_guard<std::mutex> const lock {bus_mutex_};
    return drainPending();
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  BusStatistics CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::getStatistics() const {
    std::lock_guard<std::mutex> const lock {bus_mutex_};
    BusStatistics statistics {statistics_};
    statistics.recv_buffer_bytes = can::Node::getRecvBufferSize();
    return statistics;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  void CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::resetStatistics() {
    std::lock_guard<std::mutex> const lock {bus_mutex_};
    statistics_ = BusStatistics{};
    statistics_.recv_buffer_bytes = can::Node::getRecvBufferSize();
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  void CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::addId(std::uint32_t const actuator_id) {
    if ((actuator_id < 1) || (actuator_id > 32)) {
      throw Exception("Given actuator id '" + std::to_string(actuator_id) + "' out of admittable range [1, 32]!");
    }
    std::lock_guard<std::mutex> const lock {bus_mutex_};
    actuator_ids_.push_back(actuator_id);
    std::vector<std::uint32_t> can_receive_ids {};
    for (auto const& id: actuator_ids_){
      can_receive_ids.emplace_back(getCanReceiveId(id));
    }
    setRecvFilter(can_receive_ids);
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  void CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::send(Message const& msg, std::uint32_t const actuator_id) {
    std::lock_guard<std::mutex> const lock {bus_mutex_};
    auto const can_send_id {getCanSendId(actuator_id)};
    write(can_send_id, msg.getData());
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  std::array<std::uint8_t,8> CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::sendRecv(Message const& request, std::uint32_t const actuator_id) {
    // Held for the whole exchange so that two threads cannot have requests outstanding at the same time and end
    // up reading each other's replies.
    std::lock_guard<std::mutex> const lock {bus_mutex_};
    auto const can_send_id {getCanSendId(actuator_id)};
    auto const expected_receive_id {getCanReceiveId(actuator_id)};
    auto const expected_command {request.getData()[0]};

    // Anything already queued was received before this request was even sent, so none of it can be the reply.
    // Clearing it first is what stops one lost reply from cascading: left in place, the next transaction reads
    // the previous one's answer, mis-parses it, and hands its own answer to the transaction after that.
    drainPending();

    write(can_send_id, request.getData());

    // One deadline for the whole exchange rather than one per frame, so that skipping over a burst of broadcast
    // traffic cannot extend the wait indefinitely.
    auto const deadline {std::chrono::steady_clock::now() + response_timeout_};
    for (;;) {
      auto const now {std::chrono::steady_clock::now()};
      if (now >= deadline) {
        ++statistics_.num_timeouts;
        std::ostringstream ss {};
        ss << "No reply to command " << std::showbase << std::hex << static_cast<std::uint16_t>(expected_command)
           << std::dec << " from actuator " << actuator_id << " within "
           << std::chrono::duration_cast<std::chrono::milliseconds>(response_timeout_).count() << " ms";
        if (statistics_.num_bus_errors > 0) {
          ss << " (" << statistics_.num_bus_errors << " bus errors so far, last: " << statistics_.last_bus_error << ")";
        }
        throw TimeoutException(ss.str());
      }
      if (!can::Node::waitForData(std::chrono::duration_cast<std::chrono::microseconds>(deadline - now))) {
        continue;
      }
      can::Frame frame {0U, std::array<std::uint8_t,8>{}};
      try {
        if (!can::Node::tryRead(frame)) {
          continue;
        }
      } catch (can::BusOffError const&) {
        throw;
      } catch (can::ControllerRestartedError const&) {
        throw;
      } catch (can::Exception const& e) {
        // Arbitration losses and similar transient conditions are normal on a heavily loaded bus and say nothing
        // about whether this particular reply is coming. Record them and keep waiting rather than failing a
        // position command over another node's collision. Bus-off and controller restart are not transient and
        // are rethrown above.
        ++statistics_.num_bus_errors;
        statistics_.last_bus_error = e.what();
        continue;
      }
      ++statistics_.num_frames_received;
      if (isReplyTo(frame, expected_receive_id, expected_command)) {
        ++statistics_.num_replies_matched;
        return frame.getData();
      }
      cacheFrame(frame);
    }
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  constexpr std::uint32_t CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::getCanSendId(std::uint32_t const actuator_id) const noexcept {
    return SEND_ID_OFFSET + actuator_id;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  constexpr std::uint32_t CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::getCanReceiveId(std::uint32_t const actuator_id) const noexcept {
    return RECEIVE_ID_OFFSET + actuator_id;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  bool CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::isReplyTo(can::Frame const& frame, std::uint32_t const expected_receive_id,
                                                            std::uint8_t const expected_command) noexcept {
    if (frame.getId() != expected_receive_id) {
      return false;
    }
    if (ResponseMatching<SEND_ID_OFFSET>::has_command_echo) {
      return frame.getData()[0] == expected_command;
    }
    return true;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  void CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::cacheFrame(can::Frame const& frame) {
    auto const& data {frame.getData()};
    telemetry_[frame.getId()][data[0]] = CachedResponse{data, std::chrono::steady_clock::now()};
    ++statistics_.num_frames_cached;
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  std::size_t CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::drainPending() {
    std::size_t num_drained {0};
    can::Frame frame {0U, std::array<std::uint8_t,8>{}};
    for (;;) {
      bool has_frame {false};
      try {
        has_frame = can::Node::tryRead(frame);
      } catch (can::BusOffError const&) {
        throw;
      } catch (can::ControllerRestartedError const&) {
        throw;
      } catch (can::Exception const& e) {
        // The error frame has already been consumed, so the queue is shrinking and this cannot loop forever.
        ++statistics_.num_bus_errors;
        statistics_.last_bus_error = e.what();
        continue;
      }
      if (!has_frame) {
        break;
      }
      ++statistics_.num_frames_received;
      cacheFrame(frame);
      ++num_drained;
    }
    return num_drained;
  }

}

#endif // MYACTUATOR_RMD__DRIVER__CAN_NODE
