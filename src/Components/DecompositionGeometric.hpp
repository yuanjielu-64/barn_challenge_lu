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

#ifndef Antipatrea__DecompositionGeometric_HPP_
#define Antipatrea__DecompositionGeometric_HPP_

#include "Components/Decomposition.hpp"
#include "Components/Constants.hpp"

namespace Antipatrea
{
class DecompositionGeometric : public Decomposition
{
  public:
    DecompositionGeometric(void) : Decomposition(),
                                   m_scaleFactorForCollisionStatus(Constants::DECOMPOSITION_SCALE_FACTOR_FOR_COLLISION_STATUS)
    {
    }

    virtual ~DecompositionGeometric(void)
    {
    }

    virtual double GetScaleFactorForCollisionStatus(void) const
    {
        return m_scaleFactorForCollisionStatus;
    }

    virtual void SetScaleFactorForCollisionStatus(const double s)
    {
        m_scaleFactorForCollisionStatus = s;
    }

    virtual void SetupFromParams(Params &params)
    {
        Decomposition::SetupFromParams(params);
        SetScaleFactorForCollisionStatus(params.GetValueAsDouble(Constants::KW_ScaleFactorForCollisionStatus,
                                                                 GetScaleFactorForCollisionStatus()));
    }

    virtual void Construct(const double tmax);

    virtual bool Construct();

    virtual void DrawRegionsShape(const Flags flags);

  protected:
    virtual void AddRegions(void) = 0;
   
    virtual void ConnectRegions(void) = 0;
   
    virtual void ConnectGoals(void);
   
    double m_scaleFactorForCollisionStatus;
};
}

#endif
