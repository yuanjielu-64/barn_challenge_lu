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
#include "Components/DecompositionEdge.hpp"

namespace Antipatrea
{

void DecompositionEdge::SetValues(Simulator &sim, Region &r1, Region &r2)
{
  SetDistance(sim.DistanceCfgs(r1.GetCfg(), r2.GetCfg()));
  SetDuration(GetDistance());
  SetClearance(std::min(r1.GetClearance(), r2.GetClearance()));
  SetCost(GetDuration() / (sim.GetVelocityScaleConversion() * sim.GetMaxVelocity()));


  //auto pc = sim.PredictCost(r1.GetCfg()[0], r1.GetCfg()[1], r2.GetCfg()[0], r2.GetCfg()[1]);
  // SetCost(pc);
  
}
}
