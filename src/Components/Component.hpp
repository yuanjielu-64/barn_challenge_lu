/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */

#ifndef Antipatrea__Component_HPP_
#define Antipatrea__Component_HPP_

#include "Utils/Logger.hpp"
#include "Utils/Params.hpp"
#include <typeinfo>

namespace Antipatrea {
/**
     *@brief Base class for each motion-planning component.
     * 
     *@remarks
     * Each motion-planning component should extend this class and
     * provide additional functionality as needed.
     */
    class Component {
    public:
        Component(void) {
        }

        virtual ~Component(void) {
        }

        /**
         *@brief Return the class name associated with the object.
         */
        static const char *Name(const Component *const obj) {
            return obj ? typeid(*obj).name() : "null";
        }

        /**
         *@brief Print the paramater values and the names of the components
         *       used by this class.
         *
         *@remarks
         * - This function is useful for debugging purposes to see
         *   that the component has been setup as desired.
         */
        virtual void Info(void) const {
            Logger::m_out << Name(this) << std::endl;
        }

        /**
         *@brief Check whether or not the component has been setup correctly.
         *
         *@remarks
         * Check that parameter values and/or pointers to other components
         * or objects have been setup correctly.
         */
        virtual bool CheckSetup(void) const {
            return true;
        }

        /**
         *@brief Set the parameter values of the component from the given parameters.
         *
         *@remarks
         * For each parameter value of the component, use the associated keyword to
         * get the corresponding paramater value from <tt>params</tt>.
         */
        virtual void SetupFromParams(Params &params) {
        }
    };
}

#endif
