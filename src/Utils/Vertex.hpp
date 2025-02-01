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

#ifndef Antipatrea__Vertex_HPP_
#define Antipatrea__Vertex_HPP_

#include "Utils/Flags.hpp"

namespace Antipatrea
{
template <typename Key>
class Vertex : public FlagsContainer
{
  public:
    Vertex(void) : FlagsContainer()
    {
    }

    virtual ~Vertex(void)
    {
    }

    virtual Key GetKey(void) const
    {
        return m_key;
    }

    virtual void SetKey(const Key key)
    {
        m_key = key;
    }

  protected:
    Key m_key;
};
}

#endif
