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
#include "Components/DecompositionTriangles.hpp"
#include "SceneAndSimulation/Scene2D.hpp"
#include "External/ShewchukTriangle.hpp"

namespace Antipatrea
{
void DecompositionTriangles::AddRegions(void)
{
  dynamic_cast<Scene2D *>(GetSimulator()->GetScene())->Triangulate(m_triVertices, m_triIndices, m_triNeighs, m_triAvgArea);

  RemoveSmallTriangles(m_triVertices, m_triIndices, m_triNeighs, m_triMinArea);

  // create regions
  RegionTriangle2D *r;
  const bool clearance = true;

  const int n = m_triIndices.size() / 3;
  for (int i = 0; i < n; ++i)
  {
    const int *tri = &m_triIndices[3 * i];

    r = new RegionTriangle2D();
    r->SetKey(i);
    r->SetVertex(0, &m_triVertices[2 * tri[0]]);
    r->SetVertex(1, &m_triVertices[2 * tri[1]]);
    r->SetVertex(2, &m_triVertices[2 * tri[2]]);
    r->SetFlags(AddFlags(RemoveFlags(r->GetFlags(), Region::STATUS_COLLISION), Region::STATUS_FREE));
    CreateRepresentativeCfg(*r, clearance);
    m_graph.AddVertex(r);
  }

  Logger::m_out << "added " << n << " triangles " << std::endl;
}

void DecompositionTriangles::ConnectRegions(void)
{
  const int n = m_triIndices.size() / 3;
  for (int i = 0; i < n; ++i)
  {
    const int *neighs = &m_triNeighs[3 * i];
    auto r = dynamic_cast<RegionTriangle2D *>(m_graph.GetVertex(i));
    for (int j = 0; j < 3; ++j)
      if (r->GetKey() < neighs[j])
      {
        auto rneigh = dynamic_cast<RegionTriangle2D *>(m_graph.GetVertex(neighs[j]));

        auto e1 = new DecompositionEdge();
        e1->SetFromToVertexKeys(r->GetKey(), neighs[j]);
        e1->SetValues(*GetSimulator(), *r, *rneigh);
        m_graph.AddEdge(e1);

        auto e2 = new DecompositionEdge();
        e2->SetFromToVertexKeys(neighs[j], r->GetKey());
        e2->CopyValuesFrom(*e1);
        m_graph.AddEdge(e2);
      }
  }
}

Id DecompositionTriangles::LocateRegion(const double cfg[])
{
  if (m_cellsToRegions.size() == 0)
    LocatorCompleteSetup();

  double pos[3];

  GetSimulator()->GetPositionCfg(cfg, pos);

  const int cid = GetSimulator()->GetScene()->GetGrid()->GetCellId(pos);

  if (cid < 0 || cid >= GetSimulator()->GetScene()->GetGrid()->GetNrCells())
    return Constants::ID_UNDEFINED;

  if (m_cellsToRegions[cid]->m_insideRegion >= 0)
    return m_cellsToRegions[cid]->m_insideRegion;

  for (int i = m_cellsToRegions[cid]->m_intersectRegions.size() - 1; i >= 0; --i)
  {
    const int rid = m_cellsToRegions[cid]->m_intersectRegions[i];
    const double *poly = dynamic_cast<RegionTriangle2D *>(m_graph.GetVertex(rid))->GetVertices();

    if (IsPointInsideTriangle2D(pos, &poly[0], &poly[2], &poly[4]))
      return rid;
  }
  return Constants::ID_UNDEFINED;
}

void DecompositionTriangles::LocatorCompleteSetup(void)
{
  DeleteItems<InsideIntersectRegions *>(m_cellsToRegions);
  m_cellsToRegions.clear();

  const int nrGridCells = GetSimulator()->GetScene()->GetGrid()->GetNrCells();
  m_cellsToRegions.resize(nrGridCells);
  for (int i = 0; i < nrGridCells; ++i)
  {
    m_cellsToRegions[i] = new InsideIntersectRegions();
    m_cellsToRegions[i]->m_insideRegion = -1;
  }

  for (int i = (m_triIndices.size() / 3) - 1; i >= 0; --i)
    LocatorUpdateCells(*(dynamic_cast<RegionTriangle2D *>(m_graph.GetVertex(i))));
}

void DecompositionTriangles::LocatorUpdateCells(RegionTriangle2D &r)
{
  Polygon2D poly;
  std::vector<int> cellsInside;
  std::vector<int> cellsIntersect;

  poly.AddVertex(r.GetVertex(0));
  poly.AddVertex(r.GetVertex(1));
  poly.AddVertex(r.GetVertex(2));
  poly.OccupiedGridCells(*(GetSimulator()->GetScene()->GetGrid()), cellsInside, cellsIntersect);

  for (int j = cellsInside.size() - 1; j >= 0; --j)
    m_cellsToRegions[cellsInside[j]]->m_insideRegion = r.GetKey();
  for (int j = cellsIntersect.size() - 1; j >= 0; --j)
    m_cellsToRegions[cellsIntersect[j]]->m_intersectRegions.push_back(r.GetKey());
}
}
