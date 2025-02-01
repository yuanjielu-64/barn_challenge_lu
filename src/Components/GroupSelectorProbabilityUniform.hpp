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

#ifndef Antipatrea__GroupSelectorProbabilityUniform_HPP_
#define Antipatrea__GroupSelectorProbabilityUniform_HPP_

#include "Components/GroupSelector.hpp"
#include "Utils/PseudoRandom.hpp"

namespace Antipatrea
{

class GroupSelectorProbabilityUniform : public GroupSelector
{
  public:
    GroupSelectorProbabilityUniform(void) : GroupSelector()
    {
    }

    virtual ~GroupSelectorProbabilityUniform(void)
    {
    }

    virtual Group* SelectGroup(void)
    {
        return GetGroups()->GetGroup(RandomUniformInteger(0, GetGroups()->GetNrGroups() - 1));
    }

 };
}

#endif
