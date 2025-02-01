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
#include "Utils/GManagerComponent.hpp"
#include "Utils/GManager.hpp"

namespace Antipatrea
{

bool GManagerComponent::HandleEventOnMenu(const int item)
{
    if (item >= m_menuFirstItem && item <= m_menuLastItem)
    {
        auto items = GetManager()->GetMenuItems();

        if (HasAllFlags((*items)[item]->GetFlags(), GMenuItem::FLAG_TOGGLES))
        {
            GetManager()->SetMenu(m_menu);
            (*items)[item]->SetFlags(FlipFlags((*items)[item]->GetFlags(), GMenuItem::FLAG_ON));
            GetManager()->ChangeToMenuEntry(item - m_menuFirstItem + 1, (*items)[item]->GetExtendedName(), item);
            return true;
        }
    }

    return false;
}

int GManagerComponent::PrepareMenu(void)
{
    m_menu = GetManager()->CreateMenu();

    return m_menu;
}
}
