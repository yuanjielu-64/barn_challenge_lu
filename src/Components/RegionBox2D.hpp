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

#ifndef Antipatrea__RegionBox2D_HPP_
#define Antipatrea__RegionBox2D_HPP_

#include "Components/Region2D.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Geometry.hpp"

namespace Antipatrea
{

class RegionBox2D : public Region2D
{
public:
  RegionBox2D(void)
      : Region2D()
  {
    m_box[0] = m_box[1] = m_box[2] = m_box[3] = 0.0;
  }

  virtual ~RegionBox2D(void)
  {
  }

  virtual const double *GetMinMax(void) const
  {
    return m_box;
  }

  virtual const double *GetMin(void) const
  {
    return m_box;
  }

  virtual const double *GetMax(void) const
  {
    return &m_box[2];
  }

  virtual void SetMin(const double xmin, const double ymin)
  {
    m_box[0] = xmin;
    m_box[1] = ymin;
  }

  virtual void SetMin(const double min[])
  {
    SetMin(min[0], min[1]);
  }

  virtual void SetMax(const double xmax, const double ymax)
  {
    m_box[2] = xmax;
    m_box[3] = ymax;
  }

  virtual void SetMax(const double max[])
  {
    SetMax(max[0], max[1]);
  }

  virtual void SetMinMax(const double xmin, const double ymin, const double xmax, const double ymax)
  {
    SetMin(xmin, ymin);
    SetMax(xmax, ymax);
  }

  virtual void SetMinMax(const double min[], const double max[])
  {
    SetMin(min[0], min[1]);
    SetMax(max[0], max[1]);
  }

  virtual void SetMinMax(const double minmax[])
  {
    SetMin(minmax[0], minmax[1]);
    SetMax(minmax[2], minmax[3]);
  }

  virtual void SetupFromParams(Params &params)
  {
    params.GetValuesAsDoubles(Constants::KW_Box2D, m_box, 4);
  }

  virtual void GetRepresentativePoint(double p[])
  {
    p[0] = 0.5 * (GetMin()[0] + GetMax()[0]);
    p[1] = 0.5 * (GetMin()[1] + GetMax()[1]);
  }

  virtual bool IsPointInside(const double p[])
  {
    return IsPointInsideBox2D(p, GetMin(), GetMax());
  }

  virtual void SamplePointInside(double p[])
  {
    SampleRandomPointInsideBox2D(GetMin(), GetMax(), p);
  }

  virtual void AddToTriMesh(TriMesh &tmesh)
  {
    tmesh.AddBox2D(m_box[0], m_box[1], m_box[2], m_box[3]);
  }

  virtual void DrawShape(void)
  {
    GDrawBox2D(m_box);
  }

protected:
  double m_box[4];
};
}

#endif
