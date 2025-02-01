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

#ifndef Antipatrea__TourGeneratorRandom_HPP_
#define Antipatrea__TourGeneratorRandom_HPP_

#include "Components/TourGenerator.hpp"

namespace Antipatrea
{

class TourGeneratorRandom : public TourGenerator
{
  public:
    TourGeneratorRandom(void) : TourGenerator()
    {
    }

    virtual ~TourGeneratorRandom(void)
    {
    }

    virtual bool GenerateTour(Tour &tour);
};
}

#endif
