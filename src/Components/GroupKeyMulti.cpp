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
#include "Components/GroupKeyMulti.hpp"

namespace Antipatrea
{

void GroupKeyMulti::SetContent(const GroupKey &key)
{
    const int n = m_keys.size();

    auto others = dynamic_cast<const GroupKeyMulti &>(key).GetKeys();

    for (int i = 0; i < n; ++i)
        m_keys[i]->SetContent(*((*others)[i]));
}

void GroupKeyMulti::SetContent(const Problem &prob)
{
    for (auto &curr : m_keys)
        curr->SetContent(prob);
}

void GroupKeyMulti::UpdateContent(const MPTreeVertex &v)
{
    for (auto &curr : m_keys)
        curr->UpdateContent(v);
}

bool GroupKeyMulti::SameContent(const GroupKey &key) const
{
    const int n = m_keys.size();

    auto others = dynamic_cast<const GroupKeyMulti &>(key).GetKeys();

    if (n != others->size())
        return false;

    for (int i = 0; i < n; ++i)
        if (m_keys[i]->SameContent(*((*others)[i])) == false)
            return false;
    return true;
}

bool GroupKeyMulti::IsSolved(void) const
{
    for (auto &curr : m_keys)
        if (curr->IsSolved())
            return true;
    return false;
}
}
