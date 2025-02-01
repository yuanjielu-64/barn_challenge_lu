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

#ifndef Antipatrea__GroupSelector_HPP_
#define Antipatrea__GroupSelector_HPP_

#include "Components/Groups.hpp"
#include "Components/Component.hpp"

namespace Antipatrea {

    class GroupSelector : public Component,
                          public GroupsContainer {
    public:
        GroupSelector(void) : Component(),
                              GroupsContainer() {
        }

        virtual ~GroupSelector(void) {
        }

        virtual bool CheckSetup(void) const {
            return Component::CheckSetup() &&
                   GetGroups();
        }

        virtual Group *SelectGroup(void) = 0;
    };

    ClassContainer(GroupSelector, m_groupSelector);
}

#endif
