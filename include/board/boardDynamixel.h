/** @file boardDynamixel.h
 *
 * implementation - Board with usb2dynamixel
 *
 */

#ifndef BOARDDYNAMIXEL_H_INCLUDED
#define BOARDDYNAMIXEL_H_INCLUDED

#include "board.h"
#include <iostream>
#include <chrono>
#include <memory>
#include <mutex>

namespace controller {
    /// create a single board controller (with usb2dynamixel)
    Board* createBoardDynamixel(void);
};

using namespace controller;

/// Board implementation
class BoardDynamixel : public Board {
    public:
        /// Pointer
        typedef std::unique_ptr<BoardDynamixel> Ptr;

        /// Construction
        BoardDynamixel(void);

        /// Destructor
        ~BoardDynamixel(void);

        /// Set reference position value for servomotor, returns error value
        unsigned int setPosition(unsigned char legNo, unsigned char jointNo, float_type angle);
        /// Set reference position value for servomotors, returns error value
        unsigned int setPosition(unsigned char legNo, const std::vector<float_type>& angle);
        /// Set reference position value for servomotors, returns error value
        unsigned int setPosition(const std::vector<float_type>& angle);

        /// Set reference speed value for servomotor, returns error value
        unsigned int setSpeed(unsigned char legNo, unsigned char jointNo, float_type speed);
        /// Set reference speed value for servomotors, returns error value
        unsigned int setSpeed(unsigned char legNo, const std::vector<float_type>& speed);
        /// Set reference speed value for servomotors, returns error value
        unsigned int setSpeed(const std::vector<float_type>& speed);

        /// Set compliance margin [0,254]- dead zone -- for this area the torque is zero, returns error value
        unsigned int setComplianceMargin(unsigned char legNo, unsigned char jointNo, float_type margin);
        /// Set compliance margins [0,254]- dead zone -- for this area the torque is zero, returns error value
        unsigned int setComplianceMargin(unsigned char legNo, const std::vector<float_type> margin);
        /// Set compliance margins [0,254]- dead zone -- for this area the torque is zero, returns error value
        unsigned int setComplianceMargin(const std::vector<float_type> margin);

        /// Set compiance slope [1,254] - the area with the reduced torque, returns error value
        unsigned int setComplianceSlope(unsigned char legNo, unsigned char jointNo, float_type slope);
        /// Set compiance slope [1,254] - the area with the reduced torque, returns error value
        unsigned int setComplianceSlope(unsigned char legNo, const std::vector<float_type>& slope);
        /// Set compiance slope [1,254] - the area with the reduced torque, returns error value
        unsigned int setComplianceSlope(const std::vector<float_type>& slope);

        /// Set torque limit torque_limit [0,1023] - the torque limit, returns error value
        unsigned int setTorqueLimit(unsigned char legNo, unsigned char jointNo, float_type torqueLimit);
        /// Set torque limit torque_limit [0,1023] - the torque limit, returns error value
        unsigned int setTorqueLimit(unsigned char legNo, const std::vector<float_type>& torqueLimit);
        /// Set torque limit torque_limit [0,1023] - the torque limit, returns error value
        unsigned int setTorqueLimit(const std::vector<float_type>& torqueLimit);

        /// Returns current position of the servomotor, returns error value
        unsigned int readPosition(unsigned char legNo, unsigned char jointNo, float_type& angle);
        /// Returns current position of the servomotors, returns error value
        unsigned int readPositions(unsigned char legNo, const std::vector<float_type>& angle);
        /// Returns current position of the servomotor, returns error value
        unsigned int readPosition(const std::vector<float_type>& angle);

        /// Returns contact force from 1-axis force sensor
        unsigned int readForce(unsigned char legNo, float_type& contactForce);
        /// Returns contact force from 1-axis force sensor
        unsigned int readForce(const std::vector<float_type>& contactForce);

        /// Returns contact force from 3-axis torque/force sensor
        unsigned int readTorqueForce(unsigned char legNo, TorqueForce& valueTF);
        /// Returns contact force from 3-axis torque/force sensor
        unsigned int readTorqueForce(const std::vector<float_type>& valueTF);

        /// Returns contact or from microswitch
        bool readContact(unsigned char legNo);
        /// Returns contact or from microswitches
        void readContact(const std::vector<bool> contact);

        /// Returns current from servo
        unsigned int readCurrent(unsigned char legNo, unsigned char jointNo, float_type& servoCurrent);
        /// Returns current from servo
        unsigned int readCurrent(unsigned char legNo, const std::vector<float_type>& servoCurrent);
        /// Returns current from servo
        unsigned int readCurrent(const std::vector<float_type>& servoCurrent);

        /// Returns torque/load from servo
        unsigned int readTorque(unsigned char legNo, unsigned char jointNo, float_type& servoTorque);
        /// Returns torque/load from servo
        unsigned int readTorque(unsigned char legNo, const std::vector<float_type>& servoTorque);
        /// Returns torque/load from servo
        unsigned int readTorque(const std::vector<float_type>& servoTorque);

        /// Set servo Offset
        void setOffset(unsigned char legNo, unsigned char jointNo, float_type offset);
        /// Set servo Offset
        void setOffset(unsigned char legNo, const std::vector<float_type> offset);
        /// Set servo Offset
        void setOffset(const std::vector<float_type> offset);

        /// Board configuration -- set default value
        void setDefault(void);

    private:


};

#endif // BOARDDYNAMIXEL_H_INCLUDED
