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

#ifndef Antipatrea__DecompositionTriangles_HPP_
#define Antipatrea__DecompositionTriangles_HPP_

#include "Components/DecompositionGeometric.hpp"
#include "Components/RegionTriangle2D.hpp"
#include <vector>

namespace Antipatrea
{

class DecompositionTriangles : public DecompositionGeometric
{
public:
  DecompositionTriangles(void)
      : DecompositionGeometric()
      , m_triMinArea(Constants::DECOMPOSITION_TRIANGLES_TRIANGLE_MIN_AREA)
      , m_triAvgArea(Constants::DECOMPOSITION_TRIANGLES_TRIANGLE_AVG_AREA)
  {
  }

  virtual ~DecompositionTriangles(void)
  {
  }

  virtual double GetTriangleMinArea(void) const
  {
    return m_triMinArea;
  }

  virtual double GetTriangleAvgArea(void) const
  {
    return m_triAvgArea;
  }

  virtual void SetTriangleMinArea(const double a)
  {
    m_triMinArea = a;
  }

  virtual void SetTriangleAvgArea(const double a)
  {
    m_triAvgArea = a;
  }

  void SetupFromParams(Params &params)
  {
    DecompositionGeometric::SetupFromParams(params);

    SetTriangleMinArea(params.GetValueAsDouble(Constants::KW_TriangleMinArea, GetTriangleMinArea()));
    SetTriangleAvgArea(params.GetValueAsDouble(Constants::KW_TriangleAvgArea, GetTriangleAvgArea()));
  }

  virtual Id LocateRegion(const double cfg[]);

protected:
  virtual void AddRegions(void);

  virtual void ConnectRegions(void);

  virtual void LocatorCompleteSetup(void);
  virtual void LocatorUpdateCells(RegionTriangle2D &r);

  struct InsideIntersectRegions
  {
    int m_insideRegion;
    std::vector<int> m_intersectRegions;
  };

  std::vector<double> m_triVertices;
  std::vector<int> m_triIndices;
  std::vector<int> m_triNeighs;
  double m_triAvgArea;
  double m_triMinArea;
  std::vector<InsideIntersectRegions *> m_cellsToRegions;
};
}

#endif
