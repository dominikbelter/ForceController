/** @file Localizer.h
 *
 * Localizer interface
 * @author: Dominik Belter
 */

#ifndef _LOCALIZER_H_
#define _LOCALIZER_H_

#include "../defs/defs.h"
#include <iostream>
#include <string>
#include <vector>

namespace controller {
    /// Localizer interface
    class Localizer{
        public:

            /// Localizer type
            enum Type {
                    /// AHRS MTi from xsense
                    TYPE_AHRS_XSENSE,
            };

            /// overloaded constructor
            Localizer(const std::string _name, Type _type) : name(_name), type(_type) {}

            /// Name of the Localizer
            virtual const std::string& getName() const { return name; }

            /// get body orientation
            virtual void getBodyOrientation(double& roll, double& pitch, double& yaw) = 0;

        protected:
            /// Localizer type
            Type type;

            /// Localizer name
            const std::string name;

            /// The state of the controller
            Mat34 bodyState;
    };
}

#endif // _Localizer_H_
