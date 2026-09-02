/**
 * \file node.hpp
 * \mainpage
 *    Contains a basic CAN interface class wrapping SocketCAN
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__CAN__NODE
#define MYACTUATOR_RMD__CAN__NODE
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "myactuator_rmd/can/frame.hpp"


namespace myactuator_rmd {
  namespace can {

    /**\class Node
     * \brief
     *     Base class for sending and receiving CAN frames over SocketCAN with a default 8*uint8 length
     *     in a blocking manner
    */
    class Node {
      public:
        /**\fn Node
         * \brief
         *    Class constructor, initialises the socket that should be used for communication and sets up debugging
         *
         * \param[in] ifname
         *    The name of the network interface that should communicated over
         * \param[in] send_timeout
         *    The send timeout for the underlying socket
         * \param[in] receive_timeout
         *    The receive timeout for the underlying socket
         * \param[in] is_signal_errors
         *    Boolean flags indicating whether error frames should be received or not
        */
        Node(std::string const& ifname, std::chrono::microseconds const& send_timeout = std::chrono::seconds(1),
             std::chrono::microseconds const& receive_timeout = std::chrono::seconds(1), bool const is_signal_errors = true);
        Node() = delete;
        // The socket file descriptor is an owned resource: copying a Node would hand the same descriptor to two
        // owners and close it twice, so copying is forbidden and moving explicitly transfers ownership.
        Node(Node const&) = delete;
        Node& operator = (Node const&) = delete;
        Node(Node&& other) noexcept;
        Node& operator = (Node&& other) noexcept;
        ~Node();

        /**\fn setLoopback
         * \brief
         *    Set the socket to also receive its own messages, this can be desirable for debugging
         *
         * \param[in] is_loopback
         *    If set to true the node will also receive its own messages
        */
        void setLoopback(bool const is_loopback);

        /**\fn setRecvFilter
         * \brief
         *    Set a filter for receiving CAN frames only for specific IDs
         *
         * \param[in] can_ids
         *    The CAN id that should be accepted (discarded if \p is_invert set to true)
         * \param[in] is_invert
         *    Invert the CAN id filter: If set to true all messages of the given ID are discarded
        */
        void setRecvFilter(std::vector<std::uint32_t> const& can_ids, bool const is_invert = false);

        /**\fn setSendTimeout
         * \brief
         *    Set socket timeout for sending frames
         *
         * \param[in] timeout
         *    Timeout that the socket should be set to for sending frames
        */
        void setSendTimeout(std::chrono::microseconds const& timeout);

        /**\fn setRecvTimeout
         * \brief
         *    Set socket timeout for receiving frames
         *
         * \param[in] timeout
         *    Timeout that the socket should be set to for receiving frames
        */
        void setRecvTimeout(std::chrono::microseconds const& timeout);

        /**\fn setErrorFilters
         * \brief
         *    Set error filters for the socket. We will only receive error frames if we explicitly activate it!
         *
         * \param[in] is_signal_errors
         *    Boolean flags indicating whether errors should be received or not
        */
        void setErrorFilters(bool const is_signal_errors);

        /**\fn setRecvBufferSize
         * \brief
         *    Request a larger kernel receive buffer for this socket.
         *
         *    The default (net.core.rmem_default, commonly 212992 bytes) only holds a few hundred CAN frames once
         *    the per-skb accounting overhead is taken into account. On a bus where many actuators broadcast
         *    active replies that is well under a second of traffic, after which the kernel silently drops frames
         *    and the replies we are waiting for disappear. Growing the buffer gives the application room to fall
         *    behind and catch up again without losing anything.
         *
         *    This is best effort and never throws: SO_RCVBUFFORCE is tried first (it bypasses the
         *    net.core.rmem_max ceiling but needs CAP_NET_ADMIN), falling back to SO_RCVBUF, which the kernel
         *    silently clamps to net.core.rmem_max. Compare the return value against the request to see what was
         *    actually granted.
         *
         * \param[in] num_bytes
         *    The requested receive buffer size in bytes
         * \return
         *    The receive buffer size in bytes that the kernel actually granted
        */
        std::size_t setRecvBufferSize(std::size_t const num_bytes) noexcept;

        /**\fn getRecvBufferSize
         * \brief
         *    Get the current kernel receive buffer size of this socket in bytes
         *
         * \return
         *    The receive buffer size in bytes, 0 if it could not be determined
        */
        [[nodiscard]]
        std::size_t getRecvBufferSize() const noexcept;

        /**\fn read
         * \brief
         *    Read a CAN frame in a blocking manner
         *    Only CAN frames that a receive filter was set for can be read
         *
         * \return
         *    The read CAN frame
        */
        [[nodiscard]]
        Frame read() const;

        /**\fn tryRead
         * \brief
         *    Read a single CAN frame without blocking
         *
         *    Unlike read() this does not wait for a frame to arrive: if the socket receive queue is empty it
         *    returns false immediately rather than blocking for up to the receive timeout. Error frames are
         *    decoded and thrown exactly as read() does.
         *
         * \param[out] frame
         *    The read CAN frame, only written to if this function returns true
         * \return
         *    True if a frame was read, false if no frame was queued
        */
        [[nodiscard]]
        bool tryRead(Frame& frame) const;

        /**\fn waitForData
         * \brief
         *    Wait until at least one CAN frame is queued for reading or the given timeout expires
         *
         *    This is used instead of the socket's own SO_RCVTIMEO so that a caller waiting for a particular
         *    reply can enforce a single deadline across an arbitrary number of intervening frames, rather than
         *    restarting a fresh timeout on every frame that turns out to be irrelevant.
         *
         * \param[in] timeout
         *    The maximum duration to wait for, a non-positive value polls without waiting
         * \return
         *    True if a frame is available for reading, false if the timeout expired first
        */
        [[nodiscard]]
        bool waitForData(std::chrono::microseconds const& timeout) const;

        /**\fn write
         * \brief
         *   Write the given CAN frame
         *
         * \param[in] frame
         *    The CAN frame to be written
        */
        void write(Frame const& frame);

        /**\fn write
         * \brief
         *    Write the given data to a CAN frame with the corresponding can_id and enqueue it
         *
         * \param[in] can_id
         *   The CAN id that the data should be sent to
         * \param[in] data
         *    The data to be sent
        */
        void write(std::uint32_t const can_id, std::array<std::uint8_t,8> const& data);

      protected:
        /**\fn initSocket
         * \brief
         *    Initialise a socket for the given network interface
         *
         * \param[in] ifname
         *    The name of the network interface that should communicated over
        */
        void initSocket(std::string const& ifname);

        /**\fn closeSocket
         * \brief
         *    Close the underlying socket
        */
        void closeSocket() noexcept;

        std::string ifname_;
        int socket_;
        std::chrono::microseconds send_timeout_;
    };

  }
}

#endif // MYACTUATOR_RMD__CAN__NODE
