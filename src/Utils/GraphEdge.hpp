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

#ifndef Antipatrea__GraphEdge_HPP_
#define Antipatrea__GraphEdge_HPP_

#include "Utils/Flags.hpp"

namespace Antipatrea
{
template <typename Key>
class GraphEdge : public FlagsContainer
{
      public:
        enum Index
        {
                INDEX_FROM = 0,
                INDEX_TO = 1
        };

        GraphEdge(void) : FlagsContainer(),
                          m_cost(0.0)
        {
        }

        virtual ~GraphEdge(void)
        {
        }

        virtual Key GetVertexKey(const Index i) const
        {
                return m_keys[i];
        }

        virtual Key GetFromVertexKey(void) const
        {
                return GetVertexKey(INDEX_FROM);
        }

        virtual Key GetToVertexKey(void) const
        {
                return GetVertexKey(INDEX_TO);
        }

        virtual double GetCost(void) const
        {
                return m_cost;
        }

        virtual void SetVertexKey(const Index i, const Key key)
        {
                m_keys[i] = key;
        }

        virtual void SetFromVertexKey(const Key key)
        {
                SetVertexKey(INDEX_FROM, key);
        }

        virtual void SetToVertexKey(const Key key)
        {
                SetVertexKey(INDEX_TO, key);
        }

        virtual void SetFromToVertexKeys(const Key keyFrom, const Key keyTo)
        {
                SetVertexKey(INDEX_FROM, keyFrom);
                SetVertexKey(INDEX_TO, keyTo);
        }

        virtual void SetCost(const double cost)
        {
                m_cost = cost;
        }

      protected:
        Key m_keys[2];
        double m_cost;
};
}

#endif
