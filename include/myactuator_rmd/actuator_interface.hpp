/**
 * \file actuator_interface.hpp
 * \mainpage
 *    Contains the interface to a single actuator
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__ACTUATOR_INTERFACE
#define MYACTUATOR_RMD__ACTUATOR_INTERFACE
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "myactuator_rmd/actuator_state/acceleration_type.hpp"
#include "myactuator_rmd/actuator_state/can_baud_rate.hpp"
#include "myactuator_rmd/actuator_state/control_mode.hpp"
#include "myactuator_rmd/actuator_state/feedback.hpp"
#include "myactuator_rmd/actuator_state/gains.hpp"
#include "myactuator_rmd/actuator_state/motor_status_1.hpp"
#include "myactuator_rmd/actuator_state/motor_status_2.hpp"
#include "myactuator_rmd/actuator_state/motor_status_3.hpp"
#include "myactuator_rmd/driver/driver.hpp"
#include "myactuator_rmd/driver/telemetry.hpp"


namespace myactuator_rmd {

  /**\class ActuatorInterface
   * \brief
   *    Actuator for commanding the MyActuator RMD actuator series
  */
  class ActuatorInterface {
    public:
      /**\fn ActuatorInterface
       * \brief
       *    Class constructor
       * 
       * \param[in] driver
       *    The driver communicating over the network interface
       * \param[in] actuator_id
       *    The actuator id [1, 32]
      */
      ActuatorInterface(Driver& driver, std::uint32_t const actuator_id);
      ActuatorInterface() = delete;
      ActuatorInterface(ActuatorInterface const&) = default;
      ActuatorInterface& operator = (ActuatorInterface const&) = default;
      ActuatorInterface(ActuatorInterface&&) = default;
      ActuatorInterface& operator = (ActuatorInterface&&) = default;

      /**\fn getAcceleration
       * \brief
       *    Reads the current acceleration
       * 
       * \return
       *    The current acceleration in dps with a resolution of 1 dps
      */
      [[nodiscard]]
      std::int32_t getAcceleration();

      /**\fn getCanId
       * \brief
       *    Get the CAN ID of the device
       * 
       * \return
       *    The CAN ID of the device starting at 0x240
      */
      [[nodiscard]]
      std::uint16_t getCanId();

      /**\fn getControllerGains
       * \brief
       *    Reads the currently used controller gains
       * 
       * \return
       *    The currently used controller gains for current, speed and position as unsigned 8-bit integers
      */
      [[nodiscard]]
      Gains getControllerGains();

      /**\fn getControlMode
       * \brief
       *    Reads the currently used control mode
       * 
       * \return
       *    The currently used control mode
      */
      [[nodiscard]]
      ControlMode getControlMode();

      /**\fn getMotorModel
       * \brief
       *    Reads the motor model currently in use by the actuator
       * 
       * \return
       *    The motor model string currently in use by the actuator, e.g. 'X8S2V10'
      */
      [[nodiscard]]
      std::string getMotorModel();

      /**\fn getMotorPower
       * \brief
       *    Reads the current motor power consumption in Watt
       * 
       * \return
       *    The current motor power consumption in Watt with a resolution of 0.1
      */
      [[nodiscard]]
      float getMotorPower();

      /**\fn getMotorStatus1
       * \brief
       *    Reads the motor status 1
       * 
       * \return
       *    The motor status 1 containing temperature, voltage and error codes
      */
      [[nodiscard]]
      MotorStatus1 getMotorStatus1();

      /**\fn getMotorStatus2
       * \brief
       *    Reads the motor status 2
       * 
       * \return
       *    The motor status 2 containing current, speed and position
      */
      [[nodiscard]]
      MotorStatus2 getMotorStatus2();

      /**\fn getMotorStatus3
       * \brief
       *    Reads the motor status 3
       * 
       * \return
       *    The motor status 3 containing detailed current information
      */
      [[nodiscard]]
      MotorStatus3 getMotorStatus3();

      /**\fn getMultiTurnAngle
       * \brief
       *    Read the multi-turn angle
       * 
       * \return
       *    The current multi-turn angle with a resolution of 0.01 deg
      */
      [[nodiscard]]
      float getMultiTurnAngle();

      /**\fn getMultiTurnEncoderPosition
       * \brief
       *    Read the multi-turn encoder position subtracted by the encoder multi-turn zero offset
       * 
       * \return
       *    The multi-turn encoder position
      */
      [[nodiscard]]
      std::int32_t getMultiTurnEncoderPosition();

      /**\fn getMultiTurnEncoderOriginalPosition
       * \brief
       *    Read the raw multi-turn encoder position without taking into consideration the multi-turn zero offset
       * 
       * \return
       *    The multi-turn encoder position
      */
      [[nodiscard]]
      std::int32_t getMultiTurnEncoderOriginalPosition();

      /**\fn getMultiTurnEncoderZeroOffset
       * \brief
       *    Read the multi-turn encoder zero offset
       * 
       * \return
       *    The multi-turn encoder zero offset
      */
      [[nodiscard]]
      std::int32_t getMultiTurnEncoderZeroOffset();

      /**\fn getRuntime
       * \brief
       *    Reads the uptime of the actuator in milliseconds
       * 
       * \return
       *    The uptime of the actuator in milliseconds
      */
      [[nodiscard]]
      std::chrono::milliseconds getRuntime();

      /**\fn getSingleTurnAngle
       * \brief
       *    Read the single-turn angle
       * \warning
       *    This does not seem to give correct values with my X8-PRO V2 actuator!
       * 
       * \return
       *    The current single-turn angle with a resolution of 0.01 deg
      */
      [[nodiscard]]
      float getSingleTurnAngle();

      /**\fn getSingleTurnEncoderPosition
       * \brief
       *    Read the single-turn encoder position
       * 
       * \return
       *    The single-turn encoder position
      */
      [[nodiscard]]
      std::int16_t getSingleTurnEncoderPosition();

      /**\fn getVersionDate
       * \brief
       *    Reads the version date of the actuator firmware
       * 
       * \return
       *    The version date of the firmware on the actuator, e.g. '20220206'
      */
      [[nodiscard]]
      std::uint32_t getVersionDate();

      /**\fn lockBrake
       * \brief
       *    Close the holding brake. The motor won't be able to turn anymore.
      */
      void lockBrake();

      /**\fn releaseBrake
       * \brief
       *    Open the holding brake leaving the motor in a movable state
      */
      void releaseBrake();

      /**\fn reset
       * \brief
       *    Reset the actuator
      */
      void reset();

      /**\fn sendCurrentSetpoint
       * \brief
       *    Send a current set-point to the actuator
       *
       * \param[in] current
       *    The current set-point in Ampere
       * \return
       *    Feedback control message containing actuator position, velocity, torque and temperature
      */
      Feedback sendCurrentSetpoint(float const current);

      /**\fn sendPositionAbsoluteSetpoint
       * \brief
       *    Send an absolute position set-point to the actuator additionally specifying a maximum velocity
       *
       * \param[in] position
       *    The position set-point in degree
       * \param[in] max_speed
       *    The maximum speed for the motion in degree per second
       * \return
       *    Feedback control message containing actuator position, velocity, torque and temperature
      */
      Feedback sendPositionAbsoluteSetpoint(float const position, float const max_speed = 500.0);

      /**\fn sendTorqueSetpoint
       * \brief
       *    Send a torque set-point to the actuator by setting the current
       *
       * \param[in] torque
       *    The desired torque in [Nm]
       * \param[in] torque_constant
       *    The motor's torque constant [Nm/A], depends on the model of the motor, refer to actuator_constants.hpp
       *    for the torque constant of your actuator
       * \return
       *    Feedback control message containing actuator position, velocity, torque and temperature
      */

      /**
       * \brief Send a force-limited position set-point
       * \param[in] position Target angle in degrees
       * \param[in] max_speed Maximum speed in dps
       * \param[in] max_torque Max torque (0-255, where 100 = 100% rated current)
       * \return Feedback status (temperature, current, speed, angle)
       */
      Feedback sendForcePositionSetpoint(float const position, float const max_speed, std::uint8_t const max_torque);


      Feedback sendTorqueSetpoint(float const torque, float const torque_constant);

      /**\fn sendVelocitySetpoint
       * \brief
       *    Send a velocity set-point to the actuator
       *
       * \param[in] speed
       *    The speed set-point in degree per second
       * \return
       *    Feedback control message containing actuator position, velocity, torque and temperature
      */
      Feedback sendVelocitySetpoint(float const speed);

      /**\fn sendCurrentSetpointNoReply
       * \brief
       *    Command a current without waiting for the acknowledgement, see sendPositionAbsoluteSetpointNoReply
       *
       * \param[in] current
       *    The current set-point in Ampere
      */
      void sendCurrentSetpointNoReply(float const current);

      /**\fn sendPositionAbsoluteSetpointNoReply
       * \brief
       *    Command an absolute position without waiting for the acknowledgement
       *
       *    For an actuator in active reply mode. Protocol section 2.35.1: "If an active reply command is set,
       *    the motor will not reply after receiving the command" -- not merely for the commands it broadcasts,
       *    but for anything. The ordinary sendPositionAbsoluteSetpoint would therefore wait out the full
       *    response timeout on every single command and then throw, so a broadcasting actuator has to be
       *    commanded through this instead.
       *
       *    There is no acknowledgement and so no way to tell whether the actuator acted on this. The state of
       *    such an actuator comes from its broadcasts, via getCachedMultiTurnAngle and the other cached
       *    accessors, and the age of those values is the only evidence that it is still alive.
       *
       * \param[in] position
       *    The absolute position set-point in degrees
       * \param[in] max_speed
       *    The maximum speed for the motion in degrees per second
      */
      void sendPositionAbsoluteSetpointNoReply(float const position, float const max_speed = 500.0);

      /**\fn sendForcePositionSetpointNoReply
       * \brief
       *    Command a force-limited position without waiting for the acknowledgement,
       *    see sendPositionAbsoluteSetpointNoReply
       *
       * \param[in] position
       *    The absolute position set-point in degrees
       * \param[in] max_speed
       *    The maximum speed for the motion in degrees per second
       * \param[in] max_torque
       *    The torque limit as a percentage of rated current [0, 255]
      */
      void sendForcePositionSetpointNoReply(float const position, float const max_speed, std::uint8_t const max_torque);

      /**\fn sendVelocitySetpointNoReply
       * \brief
       *    Command a velocity without waiting for the acknowledgement, see sendPositionAbsoluteSetpointNoReply
       *
       * \param[in] speed
       *    The velocity set-point in degrees per second
      */
      void sendVelocitySetpointNoReply(float const speed);

      /**\fn setAcceleration
       * \brief
       *    Write the acceleration/deceleration for the different modes to RAM and ROM (persistent)
       * 
       * \param[in] acceleration
       *    The desired acceleration/deceleration in dps with a resolution of 1 dps/s [100, 60000]
       *    For continuous motions the acceleration should be set to the value 0, see 
       *    https://github.com/2b-t/myactuator_rmd/issues/10#issuecomment-2195847459
       * \param[in] mode
       *    The mode of the desired acceleration/deceleration to be set
      */
      void setAcceleration(std::uint32_t const acceleration, AccelerationType const mode);

      /**\fn setCanBaudRate
       * \brief
       *    Set the communication Baud rate for CAN bus
       * 
       * \param[in] baud_rate
       *    Communication Baud rate that the actuator should operator with
      */
      void setCanBaudRate(CanBaudRate const baud_rate);

      /**\fn setCanId
       * \brief
       *    Set the CAN ID of the device
       * 
       * \param[in] can_id
       *    The CAN ID of the device in the range [1, 32]
      */
      void setCanId(std::uint16_t const can_id);

      /**\fn setCurrentPositionAsEncoderZero
       * \brief
       *    Set the zero offset (initial position) of the encoder to the current position
       * \warning
       *    Motor has be restarted in order for this to become effective
       * 
       * \return
       *    Current encoder position that was set to be zero
      */
      std::int32_t setCurrentPositionAsEncoderZero();

      /**\fn setEncoderZero
       * \brief
       *    Set the zero offset (initial position) of the encoder to a given value
       * \warning
       *    Motor has be restarted in order for this to become effective
       * 
       * \param[in] encoder_offset
       *    Encoder offset that should be set as zero
      */
      void setEncoderZero(std::int32_t const encoder_offset);

      /**\fn setControllerGains
       * \brief
       *    Write the currently used controller gains either to RAM (not persistent after reboot) or ROM (persistent)
       * 
       * \param[in] gains
       *    The PI-gains for current, speed and position to be set
       * \param[in] is_persistent
       *    Boolean argument signaling whether the controller gains should be persistent after reboot of the actuator or not
       * \return
       *    The currently used controller gains for current, speed and position as unsigned 8-bit integers
      */
      Gains setControllerGains(Gains const& gains, bool const is_persistent = false);

      /**\fn setTimeout
       * \brief
       *    Set the communication interruption protection time setting. The break will be triggered if the communication
       *    is interrupted for longer than the set time. The value 0 disables this feature.
       * 
       * \param[in] timeout
       *    The desired interruption protection time, 0 in case it should be disabled
      */
      void setTimeout(std::chrono::milliseconds const& timeout);

      /**\fn enableActiveReply
       * \brief
       *    Make the actuator broadcast a given reply frame at a fixed interval rather than only on request
       *
       *    The broadcasts arrive on this actuator's ordinary reply id carrying their ordinary command byte. The
       *    driver keeps the most recent one of each, so once this is enabled the corresponding getCached...
       *    accessor returns fresh state without any further traffic being put on the bus. That is the point of
       *    it on a crowded bus: twenty actuators polled individually cost twenty request-reply round trips per
       *    cycle, whereas twenty actuators broadcasting cost none.
       *
       *    This is sent without waiting for an acknowledgement, since the actuator's answer to it is itself
       *    indistinguishable from the broadcasts it is being asked to start.
       *
       * \param[in] reply_command
       *    The command byte to broadcast, e.g. 0x92 multi-turn angle, 0x9A status 1, 0x9C status 2
       * \param[in] interval
       *    The broadcast interval, expressed in the 10 ms resolution of the protocol
      */
      void enableActiveReply(std::uint8_t const reply_command, std::chrono::milliseconds const& interval);

      /**\fn disableActiveReply
       * \brief
       *    Stop the actuator broadcasting the given reply frame
       *
       * \param[in] reply_command
       *    The command byte that should no longer be broadcast
      */
      void disableActiveReply(std::uint8_t const reply_command);

      /**\fn getCachedResponse
       * \brief
       *    Get the most recent frame this actuator sent for a given command byte without having been asked
       *
       * \param[in] reply_command
       *    The command byte of interest
       * \return
       *    The cached frame, or no value if nothing has been received for that command
      */
      [[nodiscard]]
      std::optional<CachedResponse> getCachedResponse(std::uint8_t const reply_command) const;

      /**\fn getCachedMultiTurnAngle
       * \brief
       *    Get the multi-turn angle from the actuator's most recent 0x92 broadcast, without asking for it
       *    Check the age via getCachedResponse before relying on this, so that a silent actuator is not
       *    mistaken for a stationary one
       *
       * \return
       *    The angle in degrees, or no value if the actuator has not broadcast one
      */
      [[nodiscard]]
      std::optional<float> getCachedMultiTurnAngle() const;

      /**\fn getCachedMotorStatus1
       * \brief
       *    Get motor status 1, which carries the error flags, from the actuator's most recent 0x9A broadcast
       *
       * \return
       *    The status, or no value if the actuator has not broadcast one
      */
      [[nodiscard]]
      std::optional<MotorStatus1> getCachedMotorStatus1() const;

      /**\fn getCachedMotorStatus2
       * \brief
       *    Get motor status 2, which carries temperature, torque current and speed, from the actuator's most
       *    recent 0x9C broadcast
       *
       * \return
       *    The status, or no value if the actuator has not broadcast one
      */
      [[nodiscard]]
      std::optional<MotorStatus2> getCachedMotorStatus2() const;

      /**\fn shutdownMotor
       * \brief
       *    Turn off the motor
      */
      void shutdownMotor();

      /**\fn stopMotor
       * \brief
       *    Stop the motor if running closed loop command
      */
      void stopMotor();

    protected:
      Driver& driver_;
      std::uint32_t actuator_id_;
  };

}

#endif // MYACTUATOR_RMD__ACTUATOR_INTERFACE
