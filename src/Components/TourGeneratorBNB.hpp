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

#ifndef Antipatrea__TourGeneratorBNB_HPP_
#define Antipatrea__TourGeneratorBNB_HPP_

#include "Components/TourGenerator.hpp"

namespace Antipatrea
{

class TourGeneratorBNB : public TourGenerator
{
public:
  TourGeneratorBNB(void)
      : TourGenerator()
      , m_precision(0)
  {
  }

  virtual ~TourGeneratorBNB(void)
  {
  }

  virtual bool GenerateTour(Tour &tour);

protected:
  virtual long ToLong(const double val)
  {
    return (long)(val * pow(10, m_precision));
  }

  int m_precision;
};
}

#endif
