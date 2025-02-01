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

#ifndef Antipatrea__RegionBox3D_HPP_
#define Antipatrea__RegionBox3D_HPP_

#include "Components/Region3D.hpp"
#include "Components/Constants.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Geometry.hpp"

namespace Antipatrea
{

class RegionBox3D : public Region3D
{
public:
  RegionBox3D(void)
      : Region3D()
  {
    m_box[0] = m_box[1] = m_box[2] = m_box[3] = m_box[4] = m_box[5] = 0.0;
  }

  virtual ~RegionBox3D(void)
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
    return &m_box[3];
  }

    virtual void SetMin(const double xmin, const double ymin, const double zmin)
  {
    m_box[0] = xmin;
    m_box[1] = ymin;
    m_box[2] = zmin;    
  }

  virtual void SetMin(const double min[])
  {
      SetMin(min[0], min[1], min[2]);
  }

    virtual void SetMax(const double xmax, const double ymax, const double zmax)
  {
    m_box[3] = xmax;
    m_box[4] = ymax;
    m_box[5] = zmax;    
  }

  virtual void SetMax(const double max[])
  {
      SetMax(max[0], max[1], max[2]);
  }

    virtual void SetMinMax(const double xmin, const double ymin, const double zmin, const double xmax, const double ymax, const double zmax)
  {
      SetMin(xmin, ymin, zmin);
      SetMax(xmax, ymax, zmax);
  }

  virtual void SetMinMax(const double min[], const double max[])
  {
      SetMin(min[0], min[1], min[2]);
      SetMax(max[0], max[1], max[2]);
  }

  virtual void SetMinMax(const double minmax[])
  {
      SetMin(minmax[0], minmax[1], minmax[2]);
      SetMax(minmax[3], minmax[4], minmax[5]);
  }

  virtual void SetupFromParams(Params &params)
  {
    params.GetValuesAsDoubles(Constants::KW_Box3D, m_box, 6);
  	
  }
    
    virtual void GetDrawTextPos(double p[])
    {
	p[0] = 0.5 * (GetMin()[0] + GetMax()[0]);
	p[1] = 0.5 * (GetMin()[1] + GetMax()[1]);
	p[2] = GetMax()[2];
    }
    
  virtual void GetRepresentativePoint(double p[])
  {
    p[0] = 0.5 * (GetMin()[0] + GetMax()[0]);
    p[1] = 0.5 * (GetMin()[1] + GetMax()[1]);
    p[2] = 0.5 * (GetMin()[2] + GetMax()[2]);    
  }

  virtual bool IsPointInside(const double p[])
  {
      return
	  p[0] >= GetMin()[0] && p[0] <= GetMax()[0] &&
	  p[1] >= GetMin()[1] && p[1] <= GetMax()[1] &&
	  p[2] >= GetMin()[2] && p[2] <= GetMax()[2];
  }

  virtual void SamplePointInside(double p[])
  {
      p[0] = RandomUniformReal(GetMin()[0], GetMax()[0]);
      p[1] = RandomUniformReal(GetMin()[1], GetMax()[1]);
      p[2] = RandomUniformReal(GetMin()[2], GetMax()[2]);
   }

  virtual void AddToTriMesh(TriMesh &tmesh)
  {
      tmesh.AddBox(m_box[0], m_box[1], m_box[2], m_box[3], m_box[4], m_box[5]);
  }

  virtual void DrawShape(void)
  {
      GDrawBox3D(m_box, &m_box[3]);
  }

protected:
  double m_box[6];
};
}

#endif
