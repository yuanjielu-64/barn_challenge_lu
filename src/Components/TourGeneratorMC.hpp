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

#ifndef Antipatrea__TourGeneratorMC_HPP_
#define Antipatrea__TourGeneratorMC_HPP_

#include "Components/TourGenerator.hpp"

namespace Antipatrea
{

class TourGeneratorMC : public TourGenerator
{
public:
  TourGeneratorMC(void)
      : TourGenerator()
      , m_precision(0)
      , m_failures(0)
      , m_success(0)
      , m_depth(1)
      , m_nrCalls(0)
  {
  }

  virtual ~TourGeneratorMC(void)
  {
  }

  virtual bool GenerateTour(Tour &tour);

protected:
  virtual long ToLong(const double val)
  {
      return (long)(std::max(0.0, (val - GetStartTime())) * pow(10, m_precision));
  }

  int m_precision;
    int m_failures;
    int m_success;
    int m_depth;
    int m_nrCalls;
    
};
}

#endif
