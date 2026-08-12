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
    write(can_send_id, request.getData());

    // The receive filter set up in addId() is shared across every actuator registered on
    // this driver, so unsolicited traffic from OTHER actuators (e.g. active-angle-reply
    // broadcasts enabled by servo_listener.py) can land on the socket interleaved with the
    // reply we're actually waiting for. Discard anything that isn't from actuator_id and
    // keep reading, bounded so a reply that's genuinely missing still surfaces as an error
    // instead of silently consuming unlimited unrelated traffic forever.
    constexpr std::size_t max_discarded_frames {20};
    for (std::size_t i = 0; i < max_discarded_frames; ++i) {
      can::Frame const frame {can::Node::read()};
      if (frame.getId() == expected_receive_id) {
        return frame.getData();
      }
    }
    throw Exception("Did not receive a reply from actuator '" + std::to_string(actuator_id) + "' within "
                     + std::to_string(max_discarded_frames) + " frames -- crowded out by other actuators' traffic on the bus");
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
