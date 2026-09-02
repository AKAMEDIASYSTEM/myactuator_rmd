#include "myactuator_rmd/can/node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "myactuator_rmd/can/exceptions.hpp"
#include "myactuator_rmd/can/frame.hpp"
#include "myactuator_rmd/can/utilities.hpp"


namespace myactuator_rmd {
  namespace can {

    namespace {

      /**\fn throwIfErrorFrame
       * \brief
       *    Translate a SocketCAN error frame into the corresponding exception
       *    We will only receive these frames if the corresponding error mask is set
       *    See https://github.com/linux-can/can-utils/blob/master/include/linux/can/error.h
       *
       * \param[in] frame
       *    The raw frame that was read from the socket
      */
      void throwIfErrorFrame(struct ::can_frame const& frame) {
        if ((frame.can_id & CAN_ERR_FLAG) == 0) {
          return;
        }
        std::ostringstream ss {};
        ss << frame;
        if (frame.can_id & CAN_ERR_TX_TIMEOUT) {
          throw TxTimeoutError("Send timeout");
        } else if (frame.can_id & CAN_ERR_LOSTARB) {
          throw LostArbitrationError("CAN frame '" + ss.str() + "'");
        } else if (frame.can_id & CAN_ERR_CRTL) {
          throw ControllerProblemError("CAN frame '" + ss.str() + "'");
        } else if (frame.can_id & CAN_ERR_PROT) {
          throw ProtocolViolationError("CAN frame '" + ss.str() + "'");
        } else if (frame.can_id & CAN_ERR_TRX) {
          throw TransceiverStatusError("CAN frame '" + ss.str() + "'");
        } else if (frame.can_id & CAN_ERR_ACK) {
          throw NoAcknowledgeError("No acknowledgement from receiver");
        } else if (frame.can_id & CAN_ERR_BUSOFF) {
          throw BusOffError("Bus off");
        } else if (frame.can_id & CAN_ERR_BUSERROR) {
          throw BusError("Bus error");
        } else if (frame.can_id & CAN_ERR_RESTARTED) {
          throw ControllerRestartedError("Controller restarted");
        } else {
          throw Exception("Unknown CAN protocol error: CAN frame '" + ss.str() + "'");
        }
        return;
      }

      /**\fn toPollTimeout
       * \brief
       *    Convert a duration to the millisecond timeout expected by poll(), rounding up so that a sub-millisecond
       *    budget still gets one chance to see a frame rather than degenerating into a busy poll
       *
       * \param[in] timeout
       *    The duration to convert
       * \return
       *    The timeout in whole milliseconds, never negative (which poll() would read as 'wait forever')
      */
      int toPollTimeout(std::chrono::microseconds const& timeout) noexcept {
        if (timeout.count() <= 0) {
          return 0;
        }
        auto const milliseconds {(timeout.count() + 999)/1000};
        constexpr std::int64_t max_milliseconds {60000};
        return static_cast<int>(std::min<std::int64_t>(milliseconds, max_milliseconds));
      }

    }

    Node::Node(std::string const& ifname, std::chrono::microseconds const& send_timeout, std::chrono::microseconds const& receive_timeout,
               bool const is_signal_errors)
    : ifname_{}, socket_{-1}, send_timeout_{send_timeout} {
      initSocket(ifname);
      setSendTimeout(send_timeout);
      setRecvTimeout(receive_timeout);
      setErrorFilters(is_signal_errors);
      return;
    }

    Node::Node(Node&& other) noexcept
    : ifname_{std::move(other.ifname_)}, socket_{other.socket_}, send_timeout_{other.send_timeout_} {
      other.socket_ = -1;
      return;
    }

    Node& Node::operator = (Node&& other) noexcept {
      if (this != &other) {
        closeSocket();
        ifname_ = std::move(other.ifname_);
        socket_ = other.socket_;
        send_timeout_ = other.send_timeout_;
        other.socket_ = -1;
      }
      return *this;
    }

    Node::~Node() {
      closeSocket();
      return;
    }

    void Node::setLoopback(bool const is_loopback) {
      int const recv_own_msgs {static_cast<int>(is_loopback)};
      if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(int)) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not configure loopback");
      }
      return;
    }

    void Node::setRecvFilter(std::vector<std::uint32_t> const& can_ids, bool const is_invert) {
      std::vector<struct ::can_filter> filters {};
      filters.resize(can_ids.size());
      for (std::size_t i = 0; i < can_ids.size(); ++i) {
        auto const& can_id {can_ids[i]};
        if (is_invert) {
          filters[i].can_id = can_id | CAN_INV_FILTER;;
        } else {
          filters[i].can_id = can_id;
        }
        filters[i].can_mask = CAN_SFF_MASK;
      }
      if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(::can_filter)*filters.size()) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not configure read filter");
      }
      return;
    }

    void Node::setSendTimeout(std::chrono::microseconds const& timeout) {
      struct ::timeval const send_timeout {myactuator_rmd::toTimeval(timeout)};
      if (::setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&send_timeout), sizeof(struct ::timeval)) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error setting socket timeout");
      }
      send_timeout_ = timeout;
      return;
    }

    void Node::setRecvTimeout(std::chrono::microseconds const& timeout) {
      struct ::timeval const recv_timeout {myactuator_rmd::toTimeval(timeout)};
      if (::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&recv_timeout), sizeof(struct ::timeval)) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error setting socket timeout");
      }
      return;
    }

    void Node::setErrorFilters(bool const is_signal_errors) {
      // See https://github.com/linux-can/can-utils/blob/master/include/linux/can/error.h
      ::can_err_mask_t err_mask {};
      if (is_signal_errors) {
        err_mask = (CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB | CAN_ERR_CRTL | CAN_ERR_PROT | CAN_ERR_TRX |
                    CAN_ERR_ACK | CAN_ERR_BUSOFF | CAN_ERR_BUSERROR | CAN_ERR_RESTARTED);
      }
      if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(::can_err_mask_t)) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error setting error acknowledgement");
      }
      return;
    }

    std::size_t Node::setRecvBufferSize(std::size_t const num_bytes) noexcept {
      int const requested {static_cast<int>(num_bytes)};
      // SO_RCVBUFFORCE ignores the net.core.rmem_max ceiling but requires CAP_NET_ADMIN, so fall back to the
      // ordinary SO_RCVBUF (which the kernel silently clamps) when we are not privileged. Both are best effort:
      // a smaller-than-requested buffer degrades throughput under load but is never a correctness problem, so
      // there is nothing here worth aborting construction of the node over.
      if (::setsockopt(socket_, SOL_SOCKET, SO_RCVBUFFORCE, &requested, sizeof(int)) < 0) {
        static_cast<void>(::setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, &requested, sizeof(int)));
      }
      return getRecvBufferSize();
    }

    std::size_t Node::getRecvBufferSize() const noexcept {
      int actual {0};
      ::socklen_t len {sizeof(int)};
      if (::getsockopt(socket_, SOL_SOCKET, SO_RCVBUF, &actual, &len) < 0) {
        return 0;
      }
      if (actual < 0) {
        return 0;
      }
      // Note that the kernel reports back twice the value that was set, the surplus being reserved for its own
      // per-skb bookkeeping rather than usable payload.
      return static_cast<std::size_t>(actual);
    }

    Frame Node::read() const {
      struct ::can_frame frame {};
      if (::read(socket_, &frame, sizeof(struct ::can_frame)) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not read CAN frame");
      }
      throwIfErrorFrame(frame);
      std::array<std::uint8_t,8> data {};
      std::copy(std::begin(frame.data), std::end(frame.data), std::begin(data));
      Frame const f {frame.can_id, data};
      return f;
    }

    bool Node::tryRead(Frame& frame) const {
      struct ::can_frame raw_frame {};
      auto const num_bytes {::recv(socket_, &raw_frame, sizeof(struct ::can_frame), MSG_DONTWAIT)};
      if (num_bytes < 0) {
        if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
          return false;
        }
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not read CAN frame");
      }
      if (num_bytes != static_cast<::ssize_t>(sizeof(struct ::can_frame))) {
        // Nothing usable: report it as 'no frame' rather than handing the caller a half-populated struct.
        return false;
      }
      throwIfErrorFrame(raw_frame);
      std::array<std::uint8_t,8> data {};
      std::copy(std::begin(raw_frame.data), std::end(raw_frame.data), std::begin(data));
      frame = Frame{raw_frame.can_id, data};
      return true;
    }

    bool Node::waitForData(std::chrono::microseconds const& timeout) const {
      struct ::pollfd pfd {};
      pfd.fd = socket_;
      pfd.events = POLLIN;
      auto const num_ready {::poll(&pfd, 1, toPollTimeout(timeout))};
      if (num_ready < 0) {
        if (errno == EINTR) {
          // A signal interrupted the wait; report 'nothing yet' and let the caller re-evaluate its own deadline
          // rather than restarting a full timeout here.
          return false;
        }
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not wait for CAN frame");
      }
      return (num_ready > 0) && ((pfd.revents & POLLIN) != 0);
    }

    void Node::write(Frame const& frame) {
      return write(frame.getId(), frame.getData());
    }

    void Node::write(std::uint32_t const can_id, std::array<std::uint8_t,8> const& data) {
      struct ::can_frame frame {};
      frame.can_id = can_id;
      frame.len = 8;
      std::copy(std::begin(data), std::end(data), std::begin(frame.data));

      // A busy bus fills the interface transmit queue, at which point the kernel rejects further frames with
      // ENOBUFS immediately (it does not honour SO_SNDTIMEO for that case). Treating that as a hard failure
      // drops commands whenever many actuators are addressed back to back, so wait for the queue to drain and
      // retry within the configured send timeout instead.
      auto const deadline {std::chrono::steady_clock::now() + send_timeout_};
      for (;;) {
        auto const num_bytes {::write(socket_, &frame, sizeof(struct ::can_frame))};
        if (num_bytes == static_cast<::ssize_t>(sizeof(struct ::can_frame))) {
          return;
        }
        auto const write_errno {errno};
        auto const is_retryable {(num_bytes < 0) && ((write_errno == ENOBUFS) || (write_errno == EAGAIN) || (write_errno == EWOULDBLOCK) || (write_errno == EINTR))};
        auto const now {std::chrono::steady_clock::now()};
        if (!is_retryable || (now >= deadline)) {
          std::ostringstream ss {};
          ss << frame;
          throw SocketException(write_errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not write CAN frame '" + ss.str() + "'");
        }
        struct ::pollfd pfd {};
        pfd.fd = socket_;
        pfd.events = POLLOUT;
        static_cast<void>(::poll(&pfd, 1, toPollTimeout(std::chrono::duration_cast<std::chrono::microseconds>(deadline - now))));
      }
    }

    void Node::initSocket(std::string const& ifname) {
      ifname_ = ifname;
      socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
      if (socket_ < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error creating socket");
      }

      struct ::ifreq ifr {};
      std::strcpy(ifr.ifr_name, ifname.c_str());
      if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error manipulating device parameters");
      }

      struct ::sockaddr_can addr {};
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      if (::bind(socket_, reinterpret_cast<struct ::sockaddr*>(&addr), sizeof(addr)) < 0) {
        throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error assigning address to socket");
      }
      return;
    }

    void Node::closeSocket() noexcept {
      if (socket_ >= 0) {
        ::close(socket_);
        socket_ = -1;
      }
      return;
    }

  }
}
