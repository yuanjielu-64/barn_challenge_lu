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
#include "Components/Follow.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Geometry.hpp"

namespace Antipatrea
{

void Follow::SetupFromParams(Params &params)
{
  Component::SetupFromParams(params);
  SetWeightBase(params.GetValueAsDouble(Constants::KW_WeightBase, GetWeightBase()));
  SetSamplingBias(params.GetValueAsDouble(Constants::KW_SamplingBias, GetSamplingBias()));
  SetReachTolerance(params.GetValueAsDouble(Constants::KW_ReachTolerance, GetReachTolerance()));
  SetRadius(params.GetValueAsDouble(Constants::KW_Radius, GetRadius()));
  SetDim(params.GetValueAsInt(Constants::KW_NrDims, GetDim()));
}

void Follow::AddPoint(const double p[])
{
  const int n = GetDim();
  for (int i = 0; i < n; ++i)
    m_pts.push_back(p[i]);
}

bool Follow::IsInside(const int i, const double p[]) const
{
  const double *wpt = GetPoint(i);

  if (i == 0)
    return Algebra::PointDistanceSquared(GetDim(), p, GetPoint(i)) <= m_radius * m_radius;
  else
    return DistanceSquaredPointSegment(GetDim(), p, GetPoint(i - 1), GetPoint(i)) <= m_radius * m_radius;
}

    void Follow::Draw(void) const
    {
	if(GetDim() == 3)
	    Draw3D();
	else
	    Draw2D();	
    }

    void Follow::Draw3D(void) const
    {
	GDrawWireframe(true);
	
	const int n = GetNrPoints();
	for(int i = 0; i < n; ++i)
	    GDrawSphere3D(GetPoint(i), GetRadius());
		for(int i = 1; i < n; ++i)
		    GDrawCylinder3D(GetRadius(), GetPoint(i-1)[0],
				  GetPoint(i-1)[1],
				  GetPoint(i-1)[2],
				  GetPoint(i)[0],
				  GetPoint(i)[1],
				  GetPoint(i)[2]);
		GDrawWireframe(false);
		
	
    }
    
void Follow::Draw2D(void) const
{
  const int n = GetNrPoints();

  for (int i = 0; i < n - 1; ++i)
  {
    const double *p1 = GetPoint(i);
    const double *p2 = GetPoint(i + 1);
    const double d = GetRadius();
    const double vx = p2[0] - p1[0];
    const double vy = p2[1] - p1[1];
    const double norm = sqrt(vx * vx + vy * vy);
    const double ux = -vy * d / norm;
    const double uy = vx * d / norm;

    GDrawSegment2D(p1, p2);
    GDrawSegment2D(p1[0] + ux, p1[1] + uy, p2[0] + ux, p2[1] + uy);
    GDrawSegment2D(p1[0] - ux, p1[1] - uy, p2[0] - ux, p2[1] - uy);
  }

  GDrawWireframe(true);
  for (int i = 0; i < n; ++i)
    GDrawCircle2D(GetPoint(i), GetRadius());
  GDrawWireframe(false);

  for (int i = 0; i < n; ++i)
    GDrawCircle2D(GetPoint(i), m_reachTolerance);

  std::string msg;

  GDrawColor(0, 0, 0);
  for (int i = 0; i < n; ++i)
  {
    msg = std::to_string(i);
    GDrawString2D(msg.c_str(), GetPoint(i)[0], GetPoint(i)[1]);
  }
}
}
