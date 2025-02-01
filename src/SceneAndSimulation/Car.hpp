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

#ifndef Antipatrea__Car_HPP_
#define Antipatrea__Car_HPP_

#include "Components/Component.hpp"
#include "Utils/TriMesh.hpp"

namespace Antipatrea {

    class Car : public Component {
    public:
        Car(void);

        virtual ~Car(void) {}

        virtual double GetLengthFromEndToBackWheels(void);

        virtual double GetBodyLength(void);

        virtual double GetDimension(const int i);

        virtual const double *GetBoundingBoxMin(void) {
            return m_chassis.GetBoundingBoxMin();
        }

        virtual const double *GetBoundingBoxMax(void) {
            return m_chassis.GetBoundingBoxMax();
        }

        virtual void AdjustDimensions(const int i, const double dim);

        virtual void SetDimensions(const double dimx, const double dimy,
                                   const double dimz);

        virtual void SetAngleTire(const double a) { m_angleTire = a; }

        virtual void SetAngleSteer(const double a) { m_angleSteer = a; }

        virtual void Draw(void);

    protected:
        TriMesh m_chassis;
        TriMesh m_wheel;
        double m_bodyLengthFrac;
        double m_angleSteer;
        double m_angleTire;
    };
}

#endif
