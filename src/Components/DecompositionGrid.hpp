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

#ifndef Antipatrea__DecompositionGrid_HPP_
#define Antipatrea__DecompositionGrid_HPP_

#include "Components/DecompositionGeometric.hpp"

namespace Antipatrea
{

class DecompositionGrid : public DecompositionGeometric
{
  public:
    DecompositionGrid(void) : DecompositionGeometric()
    {
    }

    virtual ~DecompositionGrid(void)
    {
    }

    virtual Id LocateRegion(const double cfg[]);

  protected:
    virtual void AddRegions(void);
 
    virtual void ConnectRegions(void);
};
}

#endif
