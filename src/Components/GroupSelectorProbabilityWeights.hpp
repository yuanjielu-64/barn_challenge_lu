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

#ifndef Antipatrea__GroupSelectorProbabilityWeights_HPP_
#define Antipatrea__GroupSelectorProbabilityWeights_HPP_

#include "Components/GroupSelector.hpp"

namespace Antipatrea
{

class GroupSelectorProbabilityWeights : public GroupSelector
{
  public:
    GroupSelectorProbabilityWeights(void) : GroupSelector()
    {
    }

    virtual ~GroupSelectorProbabilityWeights(void)
    {
    }

    virtual Group* SelectGroup(void);

 };
}

#endif
