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
#include "Components/Groups.hpp"
#include "Utils/Constants.hpp"

namespace Antipatrea
{

Group *Groups::FindGroup(const GroupKey &key) const
{
    for (int i = m_groups.size() - 1; i >= 0; --i)
        if (m_groups[i]->GetKey() && m_groups[i]->GetKey()->SameContent(key))
            return m_groups[i];
    return NULL;
}

void Groups::UpdateSumWeights(void)
{
    m_sumWeights = 0.0;
    for (auto &group : m_groups)
        m_sumWeights += group->GetWeight();
}

}
