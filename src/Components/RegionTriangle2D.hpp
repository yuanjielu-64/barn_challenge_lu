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

#ifndef Antipatrea__RegionTriangle2D_HPP_
#define Antipatrea__RegionTriangle2D_HPP_

#include "Components/Region2D.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Geometry.hpp"

namespace Antipatrea
{

class RegionTriangle2D : public Region2D
{
public:
  RegionTriangle2D(void)
      : Region2D()
  {
    m_tri[0] = m_tri[1] = m_tri[2] = m_tri[3] = m_tri[4] = m_tri[5] = 0.0;
  }

  virtual ~RegionTriangle2D(void)
  {
  }

  virtual const double *GetVertices(void) const
  {
    return m_tri;
  }

  virtual const double *GetVertex(const int i) const
  {
    return &m_tri[2 * i];
  }

  virtual void SetVertices(const double tri[])
  {
    m_tri[0] = tri[0];
    m_tri[1] = tri[1];
    m_tri[2] = tri[2];
    m_tri[3] = tri[3];
    m_tri[4] = tri[4];
    m_tri[5] = tri[5];
  }

  virtual void SetVertex(const int i, const double v[])
  {
    m_tri[2 * i] = v[0];
    m_tri[2 * i + 1] = v[1];
  }

  virtual void SetupFromParams(Params &params)
  {
    params.GetValuesAsDoubles(Constants::KW_Triangle2D, m_tri, 8);
  }

  virtual void Draw(void)
  {
  }

  virtual void GetRepresentativePoint(double p[])
  {
    p[0] = (m_tri[0] + m_tri[2] + m_tri[4]) / 3.0;
    p[1] = (m_tri[1] + m_tri[3] + m_tri[5]) / 3.0;
  }

  virtual bool IsPointInside(const double p[])
  {
    return IsPointInsideTriangle2D(p, GetVertex(0), GetVertex(1), GetVertex(2));
  }

  virtual void SamplePointInside(double p[])
  {
    SampleRandomPointInsideTriangle2D(GetVertex(0), GetVertex(1), GetVertex(2), p);
  }

  virtual void AddToTriMesh(TriMesh &tmesh)
  {
    const double tri[] = {m_tri[0], m_tri[1], 0.0, m_tri[2], m_tri[3], 0.0, m_tri[4], m_tri[5], 0.0};

    tmesh.AddTriangle(tri);
  }

  virtual void DrawShape(void)
  {
    GDrawTriangle2D(m_tri);
  }

protected:
  double m_tri[6];
};
}

#endif
