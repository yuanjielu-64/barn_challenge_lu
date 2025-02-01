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

#ifndef Antipatrea__Region3D_HPP_
#define Antipatrea__Region3D_HPP_

#include "Components/RegionGeometric.hpp"

namespace Antipatrea
{

class Region3D : public RegionGeometric
{
public:
  Region3D(void) : RegionGeometric() {}

  virtual ~Region3D(void) {}
};
}

#endif
