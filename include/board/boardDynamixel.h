/** @file boardDynamixel.h
 *
 * implementation - Board with usb2dynamixel
 * Author1: Pawel Szczygiel
 */

#ifndef BOARDDYNAMIXEL_H_INCLUDED
#define BOARDDYNAMIXEL_H_INCLUDED

#include "board.h"
#include <iostream>
#include <chrono>
#include <memory>
#include <mutex>
#include "../3rdParty/dynamixel/dynamixel.h"

#define DEFAULT_BAUDNUM 1
#define MOVE_SERWOMOTOR 0x1E
#define P_PRESENT_POSITION_L 36
#define TORQUE 40

#define MOVING_SPEED 32
#define PRESENT_VOLTAGE 42      //used readCurrent

#define LEG_0 0
#define LEG_1 1
#define LEG_2 2
#define LEG_3 3
#define LEG_4 4
#define LEG_5 5

#define JOINT_0 0
#define JOINT_1 1
#define JOINT_2 2

#define P_MOVING_SPEED_L 32 /*!< wartosc predkosci serwomechanizmu */
#define P_CW_COMPLIANCE_MARGIN 26 /*!< blad pomiedzy zadana a aktualna pozycja, w ruchu zgodnie ze wskazowkami zegara (CW) */
#define P_CCW_COMPLIANCE_MARGIN 27 /*!< blad pomiedzy zadana a aktualna pozycja, w ruchu przeciwnym do wskazowek zegara (CCW) */
#define P_CW_COMPLIANCE_SLOPE 28 /*!< wartosc momentu serwomechanizmu gdy zbliza sie juz do zadanego polozenia, CW */
#define P_CCW_COMPLIANCE_SLOPE 29 /*!< wartosc momentu serwomechanizmu gdy zbliza sie juz do zadanego polozenia, CCW */
#define P_TORQUE_LIMIT_L 34 //maximum output torque
#define P_TEMERATURE_LIMIT_L 11 /* temperature limit*/

#define SET_TORQUE_LIMIT 34 //adres do ustawienia ograniczenia momentu
#define GET_MAX_TORQUE 14 //adres odczytu maxymalnego momentu


namespace controller {

    /// create a single board controller (with usb2dynamixel)
    Board* createBoardDynamixel(void);
};

using namespace controller;


/**
 * \brief Board implementation.
 */
class BoardDynamixel : public Board{
    public:
        /// Default values of angles for serwomotors.
        int zero_angle[18];
        //std::vector <float_type> zero_angle;  //rad
        /// Default offset values of angles for serwomotors.
        int angle_offset[18];
        //std::vector<float_type> angle_offset; //rad

        static const float_type DEG2DYNAMIXEL;
        /// Pointer
        typedef std::unique_ptr<BoardDynamixel> Ptr;

        /// Construction
        BoardDynamixel(void);

        /// Destructor
        ~BoardDynamixel(void);
        /**
         * \brief Set reference position value for servomotor.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param angle Angle value.
         * \return Return error value.
         */
        unsigned int setPosition(unsigned char legNo, unsigned char jointNo, float_type angle);
        //CDynamixel obiect;
        /**
         * \brief Set reference position value for all serwomotors in particular leg.
         * \param legNo Leg number.
         * \param &angle Vector of joint angles.
         * \return Return error value.
         */
        unsigned int setPosition(unsigned char legNo, const std::vector<float_type>& angle);
        /**
         * \brief Set reference position value for all serwomotors.
         * \param &angle Vector of joint angles.
         * \return Return error value.
         */
        unsigned int setPosition(const std::vector<float_type>& angle);
        /**
         * \brief Set reference speed value for servomotor.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param speed Speed value.
         * \return Return error value.
         */
        unsigned int setSpeed(unsigned char legNo, unsigned char jointNo, float_type speed);
        /**
         * \brief Set reference speed value for all serwomotors in particular leg.
         * \param legNo Leg number.
         * \param &speed Vector of joint speeds.
         * \return Return error value.
         */
        unsigned int setSpeed(unsigned char legNo, const std::vector<float_type>& speed);
        /**
         * \brief Set reference speed value for all serwomotors.
         * \param &speed Vector of joint speeds.
         * \return Return error value.
         */
        unsigned int setSpeed(const std::vector<float_type>& speed);
        /**
         * \brief Set compliance margin for servomotor.
         * \details [0,254]- dead zone -- for this area the torque is zero.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param margin Compliance margin.
         * \return Return error value.
         */
        unsigned int setComplianceMargin(unsigned char legNo, unsigned char jointNo, float_type margin);
        /**
         * \brief Set compliance margins for all serwomotors in particular leg.
         * \details [0,254]- dead zone -- for this area the torque is zero, returns error value
         * \param legNo Leg number
         * \param margin Vector of compliance margins.
         * \return Return error value.
         */
        unsigned int setComplianceMargin(unsigned char legNo, const std::vector<float_type> margin);
        /**
         * \brief Set compliance margins for all serwomotors.
         * \param margin Vector of compliance margins.
         * \return Return error value.
         */
        unsigned int setComplianceMargin(const std::vector<float_type> margin);


        /**
         * \brief Set compliance slope for serwomotor.
         * \details [1,254] - the area with the reduced torque
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param slope Compiance slope.
         * \return Return error value.
         */
        unsigned int setComplianceSlope(unsigned char legNo, unsigned char jointNo, float_type slope);

        /**
         * \brief Set compliance slope for all serwomotors in particular leg.
         * \details [1,254]- the area with the reduced torque.
         * \param legNo Leg number.
         * \param &slope Vector of compilace slopes.
         * \return Return error value.
         */
        unsigned int setComplianceSlope(unsigned char legNo, const std::vector<float_type>& slope);
        /**
         * \brief Set compliance slope for all serwomotors.
         * \details [1,254]- the area with the reduced torque.
         * \param &slope Vector of compilace slopes.
         * \return Return error value.
         */
        unsigned int setComplianceSlope(const std::vector<float_type>& slope);

        /**
         * \brief Set torque limit for serwomotor.
         * \details [0,1023] - the torque limit.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param torqueLimit Torque limit.
         * \return Return error value.
         */
        unsigned int setTorqueLimit(unsigned char legNo, unsigned char jointNo, float_type torqueLimit);

        /**
         * \brief Set torque limit for serwomotors in particular leg.
         * \details [0,1023] - the torque limit for servos.
         * \param legNo Leg number.
         * \param &torqueLimit Vector of Torque limits.
         * \return Return error value.
         */
        unsigned int setTorqueLimit(unsigned char legNo, const std::vector<float_type>& torqueLimit);

        /**
         * \brief Set torque limit for serwomotors.
         * \details [0,1023] - the torque limit for servos.
         * \param &torqueLimit Vector of Torque limits.
         * \return Return error value.
         */
        unsigned int setTorqueLimit(const std::vector<float_type>& torqueLimit);

        /**
         * \brief Returns current position of the servomotor.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param &angle Angle value.
         * \return Return error value.
         */
        unsigned int readPosition(unsigned char legNo, unsigned char jointNo, float_type& angle);

        /**
         * \brief Returns current position of the servomotors in particular leg.
         * \param legNo Leg number.
         * \param &angle Angle values.
         * \return Return error value.
         */
        unsigned int readPositions(unsigned char legNo, std::vector<float_type>& angle);

        /**
         * \brief Returns current position of the servomotors.
         * \param &angle Angle values.
         * \return Return error value.
         */
        unsigned int readPosition(std::vector<float_type>& angle);

        /**
         * \brief Returns contact force from 3-axis force sensor in particular leg.
         * \param legNo Leg number.
         * \param &contactForce Contact force value.
         * \return Return error value.
         */
        unsigned int readForce(unsigned char legNo, float_type& contactForce);

        // Niepotrzebna funkcja - nie ruszać.
        /**
         * \brief Returns contact forces from 3-axis force sensors.
         * \param &contactForce Contact force value.
         * \return Return error value.
         */
        unsigned int readForce(const std::vector<float_type>& contactForce);

        // Niepotrzebna funkcja - nie ruszać.
        /// Returns contact force from 3-axis torque/force sensor
        unsigned int readTorqueForce(unsigned char legNo, TorqueForce& valueTF);

        // Niepotrzebna funkcja - nie ruszać.
        /// Returns contact force from 3-axis torque/force sensor
        unsigned int readTorqueForce(const std::vector<float_type>& valueTF);

        // Niepotrzebna funkcja - nie ruszać.
        /// Returns contact or from microswitch
        bool readContact(unsigned char legNo);

        // Niepotrzebna funkcja - nie ruszać.
        /// Returns contact or from microswitches
        void readContact(const std::vector<bool> contact);

        /**
         * \brief Returns current value from servo.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param &servoCurrent Current value.
         * \return Return error value.
         */
        unsigned int readCurrent(unsigned char legNo, unsigned char jointNo, float_type& servoCurrent);

        /**
         * \brief Returns current values from servos in particular leg.
         * \param legNo Leg number.
         * \param &servoCurrent Current values.
         * \return Return error value.
         */
        unsigned int readCurrent(unsigned char legNo, std::vector<float_type>& servoCurrent);

        /**
         * \brief Returns current value from servos.
         * \param &servoCurrent Current values.
         * \return Return error value.
         */
        unsigned int readCurrent(std::vector<float_type>& servoCurrent);

        /**
         * \brief Returns torque(load) value from servo.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param &servoTorque Torque(load) value.
         * \return Return error value.
         */
        unsigned int readTorque(unsigned char legNo, unsigned char jointNo, float_type& servoTorque);

        /**
         * \brief Returns torque(load) value from servos in particular leg.
         * \param legNo Leg number.
         * \param &servoTorque Torque(load) values.
         * \return Return error value.
         */
        unsigned int readTorque(unsigned char legNo,std::vector<float_type>& servoTorque);

        /**
         * \brief Returns torque(load) value from servos.
         * \param &servoTorque Torque(load) values.
         * \return Return error value.
         */
        unsigned int readTorque(std::vector<float_type>& servoTorque);

        /**
         * \brief Returns servo's offset.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param offset Offset value.
         * \return Return error value.
         */
        void setOffset(unsigned char legNo, unsigned char jointNo, float_type offset);
        /**
         * \brief Returns offset of servos in particular leg.
         * \param legNo Leg number.
         * \param offset Vector of offset values.
         * \return Return error value.
         */
        void setOffset(unsigned char legNo, const std::vector<float_type> offset);
        /**
         * \brief Returns offset of servos.
         * \param offset Vector of offset values.
         * \return Return error value.
         */
        void setOffset(const std::vector<float_type> offset);
        /**
         * \brief Set default value.
         */
        void setDefault(void);

    private:
        /// Two objects for executing the operations
        CDynamixel dynamixelMotors[2];

};

#endif // BOARDDYNAMIXEL_H_INCLUDED
