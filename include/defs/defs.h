/** @file defs.h
*
* Controller definitions
*
*/

#ifndef DEFS_H_INCLUDED
#define DEFS_H_INCLUDED

#include <cstdint>
#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Geometry>

/// controller name space
namespace controller {

    /// controller default floating point
    typedef double float_type;

    /// 3 element vector class
    typedef Eigen::Translation<float_type,3> Vec3;

    /// Matrix representation of SO(3) group of rotations
    typedef Eigen::Matrix<float_type,3,3> Mat33;

    /// Quaternion representation of SO(3) group of rotations
    typedef Eigen::Quaternion<float_type> Quaternion;

    /// Homogeneous representation of SE(3) rigid body transformations
    typedef Eigen::Transform<double, 3, Eigen::Affine> Mat34;

    class TorqueForce {
    public:
        ///force values
        Vec3 force;

        ///torque values
        Vec3 torque;

        ///susceptibility values
        std::vector<float_type> susceptibility;

    };

    /// Board controller state
    class ControllerState{
        public:
            /// servo neutral value (middle pose)
            std::vector<float_type> neutralAngle;

            /// servo neutral value (middle pose)
            std::vector<float_type> offsetAngle;

            /// reference position
            std::vector<float_type> refAngle;

            /// reference speed
            std::vector<float_type> refSpeed;

            /// reference compliance slope
            std::vector<float_type> refComplianceSlope;

            /// reference compliance slope
            std::vector<float_type> refComplianceMargin;

            /// reference torque limit
            std::vector<float_type> refTorqueLimit;

            /// Default constructor
            inline ControllerState() {
            }
    };
}

#endif // DEFS_H_INCLUDED
