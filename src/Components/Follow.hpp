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

#ifndef Antipatrea_Follow_HPP_
#define Antipatrea_Follow_HPP_

#include "Components/Component.hpp"
#include "Components/Constants.hpp"
#include "Utils/Algebra.hpp"
#include "Utils/PseudoRandom.hpp"
#include <vector>

namespace Antipatrea
{
class Follow : public Component
{
public:
  Follow(void)
      : Component()
      , m_weightBase(Constants::FOLLOW_WEIGHT_BASE)
      , m_samplingBias(Constants::FOLLOW_SAMPLING_BIAS)
      , m_reachTolerance(Constants::FOLLOW_REACH_TOLERANCE)
      , m_radius(Constants::FOLLOW_RADIUS)
      , m_dim(2)
  {
  }

  virtual ~Follow(void)
  {
  }

  virtual double GetWeightBase(void) const
  {
    return m_weightBase;
  }

  virtual double GetSamplingBias(void) const
  {
    return m_samplingBias;
  }

  virtual double GetReachTolerance(void) const
  {
    return m_reachTolerance;
  }

  virtual double GetRadius(void) const
  {
    return m_radius;
  }

  virtual int GetDim(void) const
  {
    return m_dim;
  }

  virtual int GetNrPoints(void) const
  {
    return m_pts.size() / GetDim();
  }

  virtual const double *GetPoint(const int i) const
  {
    return &m_pts[GetDim() * i];
  }
    
  virtual void SetWeightBase(const double b)
  {
    m_weightBase = b;
  }

  virtual void SetSamplingBias(const double b)
  {
    m_samplingBias = b;
  }

  virtual void SetReachTolerance(const double tol)
  {
    m_reachTolerance = tol;
  }

  virtual void SetRadius(const double r)
  {
    m_radius = r;
  }

    virtual void SetDim(const int dim)
    {
	m_dim = dim;
    }
    
  virtual void SetupFromParams(Params &params);

  virtual void Clear(void)
  {
    m_pts.clear();
  }

  virtual void AddPoint(const double p[]);

  virtual bool IsInside(const int i, const double p[]) const;

  virtual bool Reached(const int i, const double p[]) const
  {
    return Algebra::PointDistanceSquared(GetDim(), GetPoint(i), p) <= m_reachTolerance * m_reachTolerance;
  }

  virtual double Weight(const int i) const
  {
      return pow(GetWeightBase(), i); //((double)i) / (GetNrPoints() - 1));
      // return pow(10000.0, ((double)i) / (GetNrPoints() - 1));
       
  }

  virtual void Sample(const int i, double p[])
  {
    RandomPointInsideSphere(GetDim(), GetPoint(i), RandomUniformReal() < GetSamplingBias() ? m_reachTolerance : m_radius, p);
  }

  virtual void Draw(void) const;

protected:
    virtual void Draw2D(void) const;
    virtual void Draw3D(void) const;
    
    
  int m_dim;
  std::vector<double> m_pts;
  double m_radius;

  double m_weightBase;
  double m_samplingBias;
  double m_reachTolerance;
};
}

#endif
