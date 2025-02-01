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

#ifndef Antipatrea__TreeVertex_HPP_
#define Antipatrea__TreeVertex_HPP_

#include "Utils/Vertex.hpp"

namespace Antipatrea {
    template<typename Key>
    class TreeVertex : public Vertex<Key> {
    public:
        TreeVertex(void) : Vertex<Key>(),
                           m_parent(NULL),
                           m_costEdge(0.0) {
        }

        virtual ~TreeVertex(void) {
        }

        virtual const TreeVertex<Key> *GetParent(void) const {
            return m_parent;
        }

        virtual TreeVertex<Key> *GetParent(void) {
            return m_parent;
        }

        virtual double GetEdgeCost(void) const {
            return m_costEdge;
        }

        virtual void SetParent(TreeVertex<Key> *const parent) {
            m_parent = parent;
        }

        virtual void SetEdgeCost(const double c) {
            m_costEdge = c;
        }

    protected:
        TreeVertex<Key> *m_parent;
        double m_costEdge;
    };
}

#endif
