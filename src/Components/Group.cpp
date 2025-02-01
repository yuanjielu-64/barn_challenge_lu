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
#include "Components/Group.hpp"
#include "Components/GroupKey.hpp"

namespace Antipatrea {

    Group::~Group(void) {
        if (m_key)
            delete m_key;
    }
}
