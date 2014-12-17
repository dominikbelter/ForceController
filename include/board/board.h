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
//#include "../3rdParty/dynamixel/dynamixel.h"

namespace controller {
    /// Board interface
    class Board{
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

 /**
         * \brief Set reference position value for servomotor.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param angle Angle value.
         * \return Return error value.
         */
            virtual unsigned int setPosition(unsigned char legNo, unsigned char jointNo, float_type angle) = 0;
            /**
         * \brief Set reference position value for all serwomotors in particular leg.
         * \param legNo Leg number.
         * \param &angle Vector of joint angles.
         * \return Return error value.
         */
            virtual unsigned int setPosition(unsigned char legNo, const std::vector<float_type>& angle) = 0;
            /**
         * \brief Set reference position value for all serwomotors.
         * \param &angle Vector of joint angles.
         * \return Return error value.
         */
            virtual unsigned int setPosition(const std::vector<float_type>& angle) = 0;

            /**
         * \brief Set reference speed value for servomotor.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param speed Speed value.
         * \return Return error value.
         */
            virtual unsigned int setSpeed(unsigned char legNo, unsigned char jointNo, float_type speed) = 0;
            /**
         * \brief Set reference speed value for all serwomotors in particular leg.
         * \param legNo Leg number.
         * \param &speed Vector of joint speeds.
         * \return Return error value.
         */
            virtual unsigned int setSpeed(unsigned char legNo, const std::vector<float_type>& speed) = 0;
            /**
         * \brief Set reference speed value for all serwomotors.
         * \param &speed Vector of joint speeds.
         * \return Return error value.
         */
            virtual unsigned int setSpeed(const std::vector<float_type>& speed) = 0;

            /**
         * \brief Set compliance margin for servomotor.
         * \details [0,254]- dead zone -- for this area the torque is zero.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param margin Compliance margin.
         * \return Return error value.
         */
            virtual unsigned int setComplianceMargin(unsigned char legNo, unsigned char jointNo, float_type margin) = 0;
            /**
         * \brief Set compliance margins for all serwomotors in particular leg.
         * \details [0,254]- dead zone -- for this area the torque is zero, returns error value
         * \param legNo Leg number
         * \param margin Vector of compliance margins.
         * \return Return error value.
         */
            virtual unsigned int setComplianceMargin(unsigned char legNo, const std::vector<float_type> margin) = 0;
            /**
         * \brief Set compliance margins for all serwomotors.
         * \param margin Vector of compliance margins.
         * \return Return error value.
         */
            virtual unsigned int setComplianceMargin(const std::vector<float_type> margin) = 0;

            /**
         * \brief Set compliance slope for serwomotor.
         * \details [1,254] - the area with the reduced torque
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param slope Compiance slope.
         * \return Return error value.
         */
            virtual unsigned int setComplianceSlope(unsigned char legNo, unsigned char jointNo, float_type slope) = 0;
            /**
         * \brief Set compliance slope for all serwomotors in particular leg.
         * \details [1,254]- the area with the reduced torque.
         * \param legNo Leg number.
         * \param &slope Vector of compilace slopes.
         * \return Return error value.
         */
            virtual unsigned int setComplianceSlope(unsigned char legNo, const std::vector<float_type>& slope) = 0;
            /**
         * \brief Set compliance slope for all serwomotors.
         * \details [1,254]- the area with the reduced torque.
         * \param &slope Vector of compilace slopes.
         * \return Return error value.
         */
            virtual unsigned int setComplianceSlope(const std::vector<float_type>& slope) = 0;

           /**
         * \brief Set torque limit for serwomotor.
         * \details [0,1023] - the torque limit.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param torqueLimit Torque limit.
         * \return Return error value.
         */
            virtual unsigned int setTorqueLimit(unsigned char legNo, unsigned char jointNo, float_type torqueLimit) = 0;
            /**
         * \brief Set torque limit for serwomotors.
         * \details [0,1023] - the torque limit for servos.
         * \param &torqueLimit Vector of Torque limits.
         * \return Return error value.
         */
            virtual unsigned int setTorqueLimit(unsigned char legNo, const std::vector<float_type>& torqueLimit) = 0;
            /**
         * \brief Set torque limit for serwomotors.
         * \details [0,1023] - the torque limit for servos.
         * \param &torqueLimit Vector of Torque limits.
         * \return Return error value.
         */
            virtual unsigned int setTorqueLimit(const std::vector<float_type>& torqueLimit) = 0;

              /**
         * \brief Returns current position of the servomotor.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param &angle Angle value.
         * \return Return error value.
         */
            virtual unsigned int readPosition(unsigned char legNo, unsigned char jointNo, float_type& angle) = 0;
            /**
         * \brief Returns current position of the servomotors in particular leg.
         * \param legNo Leg number.
         * \param &angle Angle values.
         * \return Return error value.
         */
            virtual unsigned int readPositions(unsigned char legNo, const std::vector<float_type>& angle) = 0;
              /**
         * \brief Returns current position of the servomotors.
         * \param &angle Angle values.
         * \return Return error value.
         */
            virtual unsigned int readPosition(const std::vector<float_type>& angle) = 0;

            /**
         * \brief Returns contact force from 3-axis force sensor in particular leg.
         * \param legNo Leg number.
         * \param &contactForce Contact force value.
         * \return Return error value.
         */
            virtual unsigned int readForce(unsigned char legNo, float_type& contactForce) = 0;
            /// Niepotrzebna funkcja - nie ruszać.
        /**
         * \brief Returns contact forces from 3-axis force sensors.
         * \param &contactForce Contact force value.
         * \return Return error value.
         */
            virtual unsigned int readForce(const std::vector<float_type>& contactForce) = 0;

           // Niepotrzebna funkcja - nie ruszać.
        /// Returns contact force from 3-axis torque/force sensor
            virtual unsigned int readTorqueForce(unsigned char legNo, TorqueForce& valueTF)= 0;
            /// Returns contact force from 3-axis torque/force sensor
            virtual unsigned int readTorqueForce(const std::vector<float_type>& valueTF)= 0;
	// Niepotrzebna funkcja - nie ruszać.
            /// Returns contact or from microswitch
            virtual bool readContact(unsigned char legNo) = 0;
// Niepotrzebna funkcja - nie ruszać.
            /// Returns contact or from microswitches
            virtual void readContact(const std::vector<bool> contact) = 0;

            /**
         * \brief Returns current value from servo.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param &servoCurrent Current value.
         * \return Return error value.
         */
            virtual unsigned int readCurrent(unsigned char legNo, unsigned char jointNo, float_type& servoCurrent) = 0;
            /**
         * \brief Returns current value from servos.
         * \param &servoCurrent Current values.
         * \return Return error value.
         */
            virtual unsigned int readCurrent(unsigned char legNo, std::vector<float_type>& servoCurrent) = 0;
            /**
         * \brief Returns current value from servos.
         * \param &servoCurrent Current values.
         * \return Return error value.
         */
            virtual unsigned int readCurrent( std::vector<float_type>& servoCurrent) = 0;

            /**
         * \brief Returns torque(load) value from servo.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param &servoTorque Torque(load) value.
         * \return Return error value.
         */
            virtual unsigned int readTorque(unsigned char legNo, unsigned char jointNo, float_type& servoTorque) = 0;
             /**
         * \brief Returns torque(load) value from servos in particular leg.
         * \param legNo Leg number.
         * \param &servoTorque Torque(load) values.
         * \return Return error value.
         */
            virtual unsigned int readTorque(unsigned char legNo, const std::vector<float_type>& servoTorque) = 0;
             /**
         * \brief Returns torque(load) value from servos.
         * \param &servoTorque Torque(load) values.
         * \return Return error value.
         */
            virtual unsigned int readTorque(const std::vector<float_type>& servoTorque) = 0;

             /**
         * \brief Returns servo's offset.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param offset Offset value.
         * \return Return error value.
         */
            virtual void setOffset(unsigned char legNo, unsigned char jointNo, float_type offset) = 0;
            /**
         * \brief Returns offset of servos in particular leg.
         * \param legNo Leg number.
         * \param offset Vector of offset values.
         * \return Return error value.
         */
            virtual void setOffset(unsigned char legNo, const std::vector<float_type> offset) = 0;
            /**
         * \brief Returns offset of servos.
         * \param offset Vector of offset values.
         * \return Return error value.
         */
            virtual void setOffset(const std::vector<float_type> offset) = 0;

            /**
         * \brief Set default value.
         */
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
