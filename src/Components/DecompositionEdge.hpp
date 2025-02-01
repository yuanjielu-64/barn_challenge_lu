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

#ifndef Antipatrea__DecompositionEdge_HPP_
#define Antipatrea__DecompositionEdge_HPP_

#include "SceneAndSimulation/Simulator.hpp"
#include "Components/Region.hpp"
#include "Utils/GraphEdge.hpp"
#include "Utils/Definitions.hpp"

namespace Antipatrea
{
class DecompositionEdge : public GraphEdge<Id>
{
      public:
        DecompositionEdge(void) : GraphEdge<Id>()
        {
                m_distance = m_duration = m_clearance = 0.0;
                SetFlags(STATUS_REGULAR);
        }

        virtual ~DecompositionEdge(void)
        {
        }

        enum
        {
                STATUS_REGULAR = 1,
                STATUS_GOAL = 2
        };

        virtual double GetDistance(void) const
        {
                return m_distance;
        }

        virtual double GetDuration(void) const
        {
                return m_duration;
        }

        virtual double GetClearance(void) const
        {
                return m_clearance;
        }

        virtual void SetDistance(const double d)
        {
                m_distance = d;
        }

        virtual void SetDuration(const double d)
        {
                m_duration = d;
        }

        virtual void SetClearance(const double clearance)
        {
                m_clearance = clearance;
        }

        virtual void CopyValuesFrom(const DecompositionEdge &edge)
        {
                SetCost(edge.GetCost());
                SetDistance(edge.GetDistance());
                SetDuration(edge.GetDuration());
                SetClearance(edge.GetClearance());
        }

        virtual void SetValues(Simulator &sim, Region &r1, Region &r2);

      protected:
        double m_distance;
        double m_duration;
        double m_clearance;
};
}

#endif
