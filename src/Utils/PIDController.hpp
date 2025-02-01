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

#ifndef Antipatrea__PIDController_HPP_
#define Antipatrea__PIDController_HPP_

#include <cstdlib>

namespace Antipatrea {
    class PIDController {
    public:
        PIDController(void) {
            Kp = 10;
            Ki = 0.1;
            Kd = 0.1;

            m_desiredValue = 0;
            m_errorIntegral = m_errorPrevious = 0;
        }

        virtual double GetDesiredValue(void) const {
            return m_desiredValue;
        }

        virtual void SetDesiredValue(const double val) {
            m_desiredValue = val;
        }

        virtual void Reset(void) {
            m_errorIntegral = m_errorPrevious = 0;
        }

        virtual double Update(const double currValue, const double dt) {
            const double error = m_desiredValue - currValue;
            m_errorIntegral += error * dt;
            const double u = Kp * error + Ki * m_errorIntegral +
                             Kd * (error - m_errorPrevious) / dt;
            m_errorPrevious = error;

            return u;
        }

        double Kp;
        double Ki;
        double Kd;

//    protected:

        double m_desiredValue;
        double m_errorIntegral;
        double m_errorPrevious;
    };
}


#endif



