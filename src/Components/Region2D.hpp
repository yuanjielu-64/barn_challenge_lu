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

#ifndef Antipatrea__Region2D_HPP_
#define Antipatrea__Region2D_HPP_

#include "Components/RegionGeometric.hpp"

namespace Antipatrea {

    class Region2D : public RegionGeometric {
    public:
        Region2D(void) : RegionGeometric() {}

        virtual ~Region2D(void) {}
    };
}

#endif
