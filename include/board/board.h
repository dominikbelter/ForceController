/** @file board.h
 *
 * Robot board interface
 *
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "../defs/defs.h"
#include <iostream>
#include <string>
#include <vector>

namespace controller {
    /// Board interface
    class Board {
        public:

            /// Board type
            enum Type {
                    /// Board with SPI interface (Messor)
                    TYPE_SPIBASED,
                    /// Board with usb2dynamixel (Messor2)
                    TYPE_USB2DYNAMIXEL,
            };

            /// overloaded constructor
            Board(const std::string _name, Type _type) : name(_name), type(_type) {};

            /// Name of the board
            virtual const std::string& getName() const { return name; }

            /// Set reference position value for servomotor, returns error value
            virtual unsigned int setPosition(unsigned char legNo, unsigned char jointNo, float_type angle) = 0;
            /// Set reference position value for servomotors, returns error value
            virtual unsigned int setPosition(unsigned char legNo, const std::vector<float_type>& angle) = 0;
            /// Set reference position value for servomotors, returns error value
            virtual unsigned int setPosition(const std::vector<float_type>& angle) = 0;

            /// Set reference speed value for servomotor, returns error value
            virtual unsigned int setSpeed(unsigned char legNo, unsigned char jointNo, float_type speed) = 0;
            /// Set reference speed value for servomotors, returns error value
            virtual unsigned int setSpeed(unsigned char legNo, const std::vector<float_type>& speed) = 0;
            /// Set reference speed value for servomotors, returns error value
            virtual unsigned int setSpeed(const std::vector<float_type>& speed) = 0;

            /// Set compliance margin [0,254]- dead zone -- for this area the torque is zero, returns error value
            virtual unsigned int setComplianceMargin(unsigned char legNo, unsigned char jointNo, float_type margin) = 0;
            /// Set compliance margins [0,254]- dead zone -- for this area the torque is zero, returns error value
            virtual unsigned int setComplianceMargin(unsigned char legNo, const std::vector<float_type> margin) = 0;
            /// Set compliance margins [0,254]- dead zone -- for this area the torque is zero, returns error value
            virtual unsigned int setComplianceMargin(const std::vector<float_type> margin) = 0;

            /// Set compiance slope [1,254] - the area with the reduced torque, returns error value
            virtual unsigned int setComplianceSlope(unsigned char legNo, unsigned char jointNo, float_type slope) = 0;
            /// Set compiance slope [1,254] - the area with the reduced torque, returns error value
            virtual unsigned int setComplianceSlope(unsigned char legNo, const std::vector<float_type>& slope) = 0;
            /// Set compiance slope [1,254] - the area with the reduced torque, returns error value
            virtual unsigned int setComplianceSlope(const std::vector<float_type>& slope) = 0;

            /// Set torque limit torque_limit [0,1023] - the torque limit, returns error value
            virtual unsigned int setTorqueLimit(unsigned char legNo, unsigned char jointNo, float_type torqueLimit) = 0;
            /// Set torque limit torque_limit [0,1023] - the torque limit, returns error value
            virtual unsigned int setTorqueLimit(unsigned char legNo, const std::vector<float_type>& torqueLimit) = 0;
            /// Set torque limit torque_limit [0,1023] - the torque limit, returns error value
            virtual unsigned int setTorqueLimit(const std::vector<float_type>& torqueLimit) = 0;

            /// Returns current position of the servomotor, returns error value
            virtual unsigned int readPosition(unsigned char legNo, unsigned char jointNo, float_type& angle) = 0;
            /// Returns current position of the servomotors, returns error value
            virtual unsigned int readPositions(unsigned char legNo, const std::vector<float_type>& angle) = 0;
            /// Returns current position of the servomotor, returns error value
            virtual unsigned int readPosition(const std::vector<float_type>& angle) = 0;

            /// Returns contact force from 1-axis force sensor
            virtual unsigned int readForce(unsigned char legNo, float_type& contactForce) = 0;
            /// Returns contact force from 1-axis force sensor
            virtual unsigned int readForce(const std::vector<float_type>& contactForce) = 0;

            /// Returns contact force from 3-axis torque/force sensor
            virtual unsigned int readTorqueForce(unsigned char legNo, TorqueForce& valueTF)= 0;
            /// Returns contact force from 3-axis torque/force sensor
            virtual unsigned int readTorqueForce(const std::vector<float_type>& valueTF)= 0;

            /// Returns contact or from microswitch
            virtual bool readContact(unsigned char legNo) = 0;
            /// Returns contact or from microswitches
            virtual void readContact(const std::vector<bool> contact) = 0;

            /// Returns current from servo
            virtual unsigned int readServoCurrent(unsigned char legNo, unsigned char jointNo, float_type& servoCurrent) = 0;
            /// Returns current from servo
            virtual unsigned int readServoCurrent(unsigned char legNo, const std::vector<float_type>& servoCurrent) = 0;
            /// Returns current from servo
            virtual unsigned int readServoCurrent(const std::vector<float_type>& servoCurrent) = 0;

            /// Returns torque/load from servo
            virtual unsigned int readServoTorque(unsigned char legNo, unsigned char jointNo, float_type& servoTorque) = 0;
            /// Returns torque/load from servo
            virtual unsigned int readServoTorque(unsigned char legNo, const std::vector<float_type>& servoTorque) = 0;
            /// Returns torque/load from servo
            virtual unsigned int readServoTorque(const std::vector<float_type>& servoTorque) = 0;

            /// Set servo Offset
            virtual void setOffset(unsigned char legNo, unsigned char jointNo, float_type offset) = 0;
            /// Set servo Offset
            virtual void setOffset(unsigned char legNo, const std::vector<float_type> offset) = 0;
            /// Set servo Offset
            virtual void setOffset(const std::vector<float_type> offset) = 0;

            /// Board configuration -- set default value
            virtual void setDefault(void) = 0;

            /// Virtual descrutor
            virtual ~Board() {}

        protected:
            /// Board type
            Type type;

            /// Board name
            const std::string name;

            /// The state of the controller
            ControllerState state;
    };
};

#endif // _BOARD_H_
