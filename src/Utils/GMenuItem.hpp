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

#ifndef Antipatrea__GMenuItem_HPP_
#define Antipatrea__GMenuItem_HPP_

#include "Utils/Flags.hpp"
#include <string>

namespace Antipatrea
{
class GMenuItem
{
  public:
    enum
    {
        FLAG_OFF = 1 << 0,
        FLAG_ON = 1 << 1,
        FLAG_TOGGLES = 1 << 2
    };

    GMenuItem(const char name[],
             const Flags flags = FLAG_ON | FLAG_TOGGLES) : m_name(name),
                                                           m_flags(flags)
    {
        UpdateExtendedName();
    }

    virtual ~GMenuItem(void)
    {
    }

    virtual const char *GetName(void) const
    {
        return m_name.c_str();
    }

    virtual const char *GetExtendedName(void) const
    {
        return m_nameExtended.c_str();
    }

    virtual const Flags GetFlags(void) const
    {
        return m_flags;
    }

    virtual void SetName(const char name[])
    {
        m_name = (std::string)name;
        UpdateExtendedName();
    }

    virtual void SetFlags(const Flags flags)
    {
        m_flags = flags;
        UpdateExtendedName();
    }

  protected:
    virtual void UpdateExtendedName(void)
    {
        if (HasAllFlags(m_flags, FLAG_TOGGLES))
            m_nameExtended = m_name + (HasAllFlags(m_flags, FLAG_ON) ? " [on]" : " [off]");
        else
            m_nameExtended = m_name;
    }

    std::string m_name;
    std::string m_nameExtended;
    Flags m_flags;
};
}

#endif
