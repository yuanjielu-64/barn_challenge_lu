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

#ifndef Antipatrea__TourGeneratorOptimal_HPP_
#define Antipatrea__TourGeneratorOptimal_HPP_

#include "Components/TourGenerator.hpp"

namespace Antipatrea
{

class TourGeneratorOptimal : public TourGenerator
{
  public:
    TourGeneratorOptimal(void) : TourGenerator()
    {
    }

    virtual ~TourGeneratorOptimal(void)
    {
    }

    virtual bool GenerateTour(Tour &tour);
};
}

#endif
