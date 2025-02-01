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
#include "Components/GroupSelectorProbabilityWeights.hpp"
#include "Utils/PseudoRandom.hpp"

namespace Antipatrea
{

Group *GroupSelectorProbabilityWeights::SelectGroup(void)
{
    auto groups = GetGroups();
    const int n = groups->GetNrGroups();
    const double r = RandomUniformReal(0, groups->GetSumWeights());
    double w = 0.0;

    for (int i = 0; i < n; ++i)
        if ((w += groups->GetGroup(i)->GetWeight()) >= r)
            return groups->GetGroup(i);
    return groups->GetGroup(n - 1);
}
}
