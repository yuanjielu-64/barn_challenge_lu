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
#include "Components/GroupSelectorMaxWeight.hpp"

namespace Antipatrea {

    Group *GroupSelectorMaxWeight::SelectGroup(void) {
        auto groups = GetGroups();
        Group *gmax = NULL;
        double wmax = -INFINITY;
        double w;

        for (int i = groups->GetNrGroups() - 1; i >= 0; --i)
            if ((w = groups->GetGroup(i)->GetWeight()) >= wmax) {
                wmax = w;
                gmax = groups->GetGroup(i);
                gmax->idx = i;
            }

        return gmax;
    }
}
