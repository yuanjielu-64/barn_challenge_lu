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
#include "Components/DecompositionGrid.hpp"
#include "Components/RegionBox2D.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/TriMeshDefault.hpp"

namespace Antipatrea {

    Id DecompositionGrid::LocateRegion(const double cfg[]) {
        double pos[3];

        GetSimulator()->GetPositionCfg(cfg, pos);
        auto id = GetSimulator()->GetScene()->GetGrid()->GetCellId(pos);

        if (id < 0 ||
            HasAnyFlags(dynamic_cast<Region *>(GetGraph()->GetVertex(id))->GetFlags(), Region::STATUS_COLLISION))
            return Constants::ID_UNDEFINED;
        return id;
    }

    void DecompositionGrid::AddRegions(void) {
        auto scene = GetSimulator()->GetScene();
        auto grid = scene->GetGrid();
        auto obsts = scene->GetObstaclesCollisionMesh();
        const int n = grid->GetNrCells();
        const int nrDims = grid->GetNrDims();
        const bool clearance = true;

        double bbox[6];
        TriMeshDefault tmesh;
        double *cfg;
        double pos[3];

        for (int i = 0; i < n; ++i) {
            grid->GetCellFromId(i, bbox);

            auto r = new RegionBox2D();
            r->SetKey(i);
            r->SetMinMax(bbox);
            CreateRepresentativeCfg(*r, clearance);
            m_graph.AddVertex(r);

            for (int j = 0; j < nrDims; ++j)
                ScaleInterval(GetScaleFactorForCollisionStatus(), bbox[j], bbox[j + nrDims]);

            tmesh.Clear();
            if (grid->GetNrDims() == 2)
                tmesh.AddBox2D(bbox[0], bbox[1], bbox[2], bbox[3]);
            else
                tmesh.AddBox(bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5]);

            if (tmesh.Collision(NULL, NULL, obsts, NULL, NULL))
                r->SetFlags(AddFlags(RemoveFlags(r->GetFlags(), Region::STATUS_FREE), Region::STATUS_COLLISION));
            else
                r->SetFlags(AddFlags(RemoveFlags(r->GetFlags(), Region::STATUS_COLLISION), Region::STATUS_FREE));
        }
    }

    void DecompositionGrid::ConnectRegions(void) {
        auto scene = GetSimulator()->GetScene();
        auto grid = scene->GetGrid();
        const int n = grid->GetNrCells();
        std::vector<int> neighs;
        Region *rneigh;

        for (int i = 0; i < n; ++i) {
            auto r = dynamic_cast<Region *>(m_graph.GetVertexByIndex(i));
            auto key = m_graph.GetVertexKey(i);
            if (r && HasAllFlags(r->GetFlags(), Region::STATUS_FREE)) {
                neighs.clear();
                grid->GetNeighbors(key, neighs);
                for (auto &neigh: neighs) {
                    rneigh = dynamic_cast<Region *>(m_graph.GetVertex(neigh));
                    if (rneigh && key < neigh && HasAllFlags(rneigh->GetFlags(), Region::STATUS_FREE)) {
                        DecompositionEdge *e1 = new DecompositionEdge();
                        e1->SetFromToVertexKeys(key, neigh);
                        e1->SetValues(*GetSimulator(), *r, *rneigh);
                        m_graph.AddEdge(e1);

                        DecompositionEdge *e2 = new DecompositionEdge();
                        e2->SetFromToVertexKeys(neigh, key);
                        e2->CopyValuesFrom(*e1);
                        m_graph.AddEdge(e2);
                    }
                }
            }
        }
    }
}
