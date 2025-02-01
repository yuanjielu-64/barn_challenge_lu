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

#ifndef Antipatrea__Flags_HPP_
#define Antipatrea__Flags_HPP_

namespace Antipatrea
{
typedef unsigned int Flags;

static inline bool HasAnyFlags(const Flags f, const Flags b)
{
    return f & b;
}

static inline bool HasAllFlags(const Flags f, const Flags b)
{
    return (f & b) == b;
}

static inline Flags AddFlags(const Flags f, const Flags b)
{
    return f | b;
}

static inline Flags RemoveFlags(const Flags f, const Flags b)
{
    return f & (~b);
}

static inline Flags RemoveAndAddFlags(const Flags f, const Flags rem, const Flags add)
{
    return (f & (~rem)) | add;
}

static inline Flags FlipFlags(const Flags f, const Flags b)
{
    return f ^ b;
}

class FlagsContainer
{
  public:
    FlagsContainer(Flags flags = 0) : m_flags(flags)
    {
    }

    virtual ~FlagsContainer(void)
    {
    }

    virtual Flags GetFlags(void) const
    {
        return m_flags;
    }

    virtual void SetFlags(const Flags flags)
    {
        m_flags = flags;
    }

  protected:
    Flags m_flags;
};
}

#endif
