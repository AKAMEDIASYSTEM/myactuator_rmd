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
#include <cstdint>
#include <string>
#include <vector>

#include "myactuator_rmd/can/frame.hpp"
#include "myactuator_rmd/can/node.hpp"
#include "myactuator_rmd/driver/driver.hpp"
#include "myactuator_rmd/protocol/message.hpp"
#include "myactuator_rmd/exceptions.hpp"


namespace myactuator_rmd {

  /**\var uses_command_byte_echo_v
   * \brief
   *    Whether replies on this CAN id range echo the request's command byte in data[0],
   *    the way the standard single-motor protocol (SingleMotorMessage) does. False for
   *    the 0x400/0x500 motion_mode range, where data[0] carries packed position data
   *    instead of a fixed command identifier -- checking it there would reject every
   *    legitimate reply, since the returned position essentially never exactly equals
   *    the high byte of the requested one.
  */
  template <std::uint32_t SEND_ID_OFFSET>
  inline constexpr bool uses_command_byte_echo_v {true};

  template <>
  inline constexpr bool uses_command_byte_echo_v<0x400> {false};

  /**\class CanNode
   * \brief
   *    Base class for the CAN driver as well as the actuator mock
  */
  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  class CanNode: public Driver, protected can::Node {
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
      CanNode& operator = (CanNode const&) = default;
      CanNode(CanNode&&) = default;
      CanNode& operator = (CanNode&&) = default;

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
      inline void send(Message const& msg, std::uint32_t const actuator_id) override;

      /**\fn sendRecv
       * \brief
       *    Writes a given CAN frame based on the request to the actuator with the corresponding id
       *    and waits for a corresponding reply
       * 
       * \param[in] request
       *    Request that should be sent to the corresponding actuator
       * \param[in] actuator_id
       *    The ID of the actuator that the message should be sent to
       * \return
       *    The response bytes
      */
      [[nodiscard]]
      inline std::array<std::uint8_t,8> sendRecv(Message const& request, std::uint32_t const actuator_id) override;

    protected:
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
      constexpr std::uint32_t getCanSendId(std::uint32_t const actuator_id) noexcept;

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
      constexpr std::uint32_t getCanReceiveId(std::uint32_t const actuator_id) noexcept;

      std::vector<std::uint32_t> actuator_ids_;
  };

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::CanNode(std::string const& ifname)
  : can::Node{ifname}, Driver{} {
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  void CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::addId(std::uint32_t const actuator_id) {
    if ((actuator_id < 1) || (actuator_id > 32)) {
      throw Exception("Given actuator id '" + std::to_string(actuator_id) + "' out of admittable range [1, 32]!");
    }
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
    auto const can_send_id {getCanSendId(actuator_id)};
    write(can_send_id, msg.getData());
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  std::array<std::uint8_t,8> CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::sendRecv(Message const& request, std::uint32_t const actuator_id) {
    auto const can_send_id {getCanSendId(actuator_id)};
    auto const expected_receive_id {getCanReceiveId(actuator_id)};
    auto const expected_command_byte {request.getData()[0]};

    // Clear out any backlog that queued up while this process was doing something else
    // (sleeping, handling other actuators, ...) BEFORE sending the request, so the read
    // loop below only has to contend with traffic that arrives after this point rather
    // than an accumulated backlog on top of it.
    can::Node::drainPending();
    write(can_send_id, request.getData());

    // Two distinct kinds of unsolicited traffic can land on the socket interleaved with
    // the reply we're actually waiting for:
    //  1. OTHER actuators' frames -- the receive filter set up in addId() is shared across
    //     every actuator registered on this driver, so it isn't ID-selective per call.
    //  2. THIS SAME actuator's own active-reply broadcasts (e.g. enabled by
    //     servo_listener.py) -- these share this actuator's reply ID, so the ID check
    //     alone can't tell them apart from the real reply. Where the protocol echoes its
    //     command byte in data[0] (see uses_command_byte_echo_v), we use that as a second
    //     check; motion_mode doesn't follow that convention, so there we only check the ID.
    // Discard anything that doesn't match and keep reading, bounded so a reply that's
    // genuinely missing still surfaces as an error instead of silently consuming
    // unlimited unrelated traffic forever. Each discarded frame here is one that was
    // already queued and ready to read (that's why it's cheap) -- a truly silent bus
    // still fails fast via can::Node::read()'s own socket-level receive timeout, which
    // fires on the very first call regardless of this bound. What this bound actually
    // limits is how long we keep discarding a *busy* bus that never produces a match.
    constexpr std::size_t max_discarded_frames {100};
    for (std::size_t i = 0; i < max_discarded_frames; ++i) {
      can::Frame const frame {can::Node::read()};
      if constexpr (uses_command_byte_echo_v<SEND_ID_OFFSET>) {
        if ((frame.getId() == expected_receive_id) && (frame.getData()[0] == expected_command_byte)) {
          return frame.getData();
        }
      } else {
        if (frame.getId() == expected_receive_id) {
          return frame.getData();
        }
      }
    }
    throw Exception("Did not receive a reply to command '" + std::to_string(expected_command_byte) + "' from actuator '"
                     + std::to_string(actuator_id) + "' within " + std::to_string(max_discarded_frames)
                     + " frames -- crowded out by other traffic on the bus");
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  constexpr std::uint32_t CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::getCanSendId(std::uint32_t const actuator_id) noexcept {
    return SEND_ID_OFFSET + actuator_id;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  constexpr std::uint32_t CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::getCanReceiveId(std::uint32_t const actuator_id) noexcept {
    return RECEIVE_ID_OFFSET + actuator_id;
  }

}

#endif // MYACTUATOR_RMD__DRIVER__CAN_NODE
