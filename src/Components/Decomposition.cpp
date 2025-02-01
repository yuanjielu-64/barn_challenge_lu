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
#include "Components/Decomposition.hpp"
#include "Utils/Colormap.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/GraphSearch.hpp"
#include "Utils/GraphSearchInfo.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include "Utils/Radix.hpp"

namespace Antipatrea {

    void Decomposition::UpdateMinMaxRegionClearances(const double c) {
        if (c > m_maxRegionClearance)
            m_maxRegionClearance = c;
        else if (c < m_minRegionClearance)
            m_minRegionClearance = c;
    }

    void Decomposition::SetInitialAndGoalsPairs(const int keyInit) {
        auto goals = GetProblem()->GetGoals();
        const int ng = goals->size();

        m_pairsInitialAndGoalsUnconnected.clear();
        for (int i = 0; i < ng; ++i) {
            m_pairsInitialAndGoalsUnconnected.push_back(keyInit);
            m_pairsInitialAndGoalsUnconnected.push_back((*goals)[i]->GetKey());

            for (int j = i + 1; j < ng; ++j) {
                m_pairsInitialAndGoalsUnconnected.push_back((*goals)[i]->GetKey());
                m_pairsInitialAndGoalsUnconnected.push_back((*goals)[j]->GetKey());
            }
        }
    }

    bool Decomposition::AreInitialAndGoalsConnected(void) {
        auto graph = GetGraph();
        int a = (int) m_pairsInitialAndGoalsUnconnected.size();
        for (int i = 0; i < (int) m_pairsInitialAndGoalsUnconnected.size();)
            if (graph->AreVerticesPathConnected(m_pairsInitialAndGoalsUnconnected[i],
                                                m_pairsInitialAndGoalsUnconnected[i + 1])) {
                m_pairsInitialAndGoalsUnconnected[i + 1] = m_pairsInitialAndGoalsUnconnected.back();
                m_pairsInitialAndGoalsUnconnected.pop_back();
                m_pairsInitialAndGoalsUnconnected[i] = m_pairsInitialAndGoalsUnconnected.back();
                m_pairsInitialAndGoalsUnconnected.pop_back();
            } else
                i += 2;
        return m_pairsInitialAndGoalsUnconnected.empty();
    }

    void Decomposition::PathsToGoalsRadix(void) {
        RadixGraph radix(GetGraph()->GetNrVertices());

        for (int i = GetGraph()->GetNrVertices() - 1; i >= 0; --i)
            radix.nodes[i]->m_id = GetGraph()->GetVertexByIndex(i)->GetKey();

        for (int i = GetGraph()->GetNrVertices() - 1; i >= 0; --i) {
            auto v = GetGraph()->GetVertexByIndex(i);
            auto edges = v->GetConnections();
            for (auto &neigh: *edges) {
                int j = radix.findId(neigh);
                if (j >= 0) {
                    auto edge = GetGraph()->FindEdge(v->GetKey(), neigh);
                    if (edge->GetCost() != INFINITY)
                        radix.addEdge(i, j, (long) (100 * edge->GetCost()));
                }
            }
        }

        ///now same as paths to goals
        Timer::Clock clk;
        Timer::Start(clk);

        static int count = 0;

        //Logger::m_out << "..computing PathsToGoals" << std::endl;

        GraphPathData<Id> *data;
        auto goals = GetProblem()->GetGoals();
        const int ng = goals->size();
        const int nv = GetGraph()->GetNrVertices();
        Id gid;
        Region *r;

        m_maxPathCosts.resize(ng);
        for (int i = 0; i < ng; ++i) {
            m_maxPathCosts[i] = 0;
            int pos = radix.findId((*goals)[i]->GetKey());
            if (pos < 0)
                exit(0);
            radix.dijkstra(radix.nodes[pos]);
            for (int j = 0; j < nv; ++j) {
                r = dynamic_cast<Region *>(GetGraph()->GetVertexByIndex(j));
                if (r != NULL) {
                    if (r->GetPathDataToGoals()->size() < ng) {
                        data = new GraphPathData<Id>();
                        r->GetPathDataToGoals()->push_back(data);
                    } else
                        data = (*(r->GetPathDataToGoals()))[j];
                    pos = radix.findId(r->GetKey());
                    if (pos < 0)
                        exit(0);

                    data->m_path.clear();
                    data->m_cost = radix.path(radix.nodes[pos], data->m_path) / 100.0;
                    if (data->m_path.size() == 0)
                        data->m_cost = INFINITY;
                    data->m_pts.clear();
                    PathFromIdsToPoints(data->m_path.size(), &(data->m_path[0]), data->m_pts);

                    if (data->m_path.size() > 0 && data->m_cost > m_maxPathCosts[i])
                        m_maxPathCosts[i] = data->m_cost;
                    //    Logger::m_out << "distance from " << r->GetKey() << " to goal " << i << " is " << data->m_cost << std::endl;

                }
            }
            Logger::m_out << "..processed goal " << i << " [max cost = " << m_maxPathCosts[i] << "]" << std::endl;
        }

        Stats::GetSingleton()->AddValue("TimePathsToGoals", Timer::Elapsed(clk));

    }

    void Decomposition::PathsToGoals() {
        PathsToGoalsRadix();
    }

    void Decomposition::AddGoals() {
        RegionGeometric *r;
        auto goals = GetProblem()->GetGoals();
        const int ng = goals->size();
        const int nv = m_graph.GetNrVertices();
        const bool clearance = true;

        for (int i = 0; i < ng; ++i) {
            r = (*goals)[i];
            r->SetKey(nv + i);
            CreateRepresentativeCfg(*r, clearance);
            m_graph.AddVertex(r);
        }
    }

    void Decomposition::CreateRepresentativeCfg(RegionGeometric &r, const bool clearance) {
        double *cfg = r.GetCfg();
        double pos[3];

        if (cfg == NULL)
            cfg = GetSimulator()->GetCfgAllocator()->New();
        GetSimulator()->SampleCfg(cfg);
        r.GetRepresentativePoint(pos);
        GetSimulator()->SetPositionCfg(pos, cfg);
        r.SetCfg(cfg);

        if (clearance) {
            r.SetClearance(GetSimulator()->ClearancePosition(pos));
            UpdateMinMaxRegionClearances(r.GetClearance());
        }
    }

    void Decomposition::DrawRegionsCfg(const Flags flags) {
        const int n = m_graph.GetNrVertices();
        for (int i = 0; i < n; ++i) {
            auto r = dynamic_cast<Region *>(m_graph.GetVertexByIndex(i));
            if (r && HasAllFlags(r->GetFlags(), flags)) {
                GDrawColor(r->GetColor());
                GetSimulator()->DrawCfg(r->GetCfg());
            }
        }
    }

    void Decomposition::DrawEdges(const Flags flags) {
        auto edges = m_graph.GetEdges();
        for (auto &iter: *edges) {
            auto edge = iter.second;
            auto idFrom = edge->GetFromVertexKey();
            auto idTo = edge->GetToVertexKey();
            if (idFrom < idTo && HasAllFlags(edge->GetFlags(), flags) && edge->GetCost() != INFINITY)
                DrawEdge(idFrom, idTo);
        }
    }

    void Decomposition::DrawEdge(const Id keyFrom, const Id keyTo) {
        auto rfrom = dynamic_cast<Region *>(m_graph.GetVertex(keyFrom));
        auto rto = dynamic_cast<Region *>(m_graph.GetVertex(keyTo));
        double pos1[3];
        double pos2[3];

        if (rfrom && rto) {
            GetSimulator()->GetPositionCfg(rfrom->GetCfg(), pos1);
            GetSimulator()->GetPositionCfg(rto->GetCfg(), pos2);

            if (GetSimulator()->GetScene()->GetGrid()->GetNrDims() == 2)
                GDrawSegment2D(pos1, pos2);
            else
                GDrawSegment3D(pos1, pos2);
        }
    }

    void Decomposition::SetRegionColors(const double r, const double g, const double b, const Flags flags) {
        const int n = m_graph.GetNrVertices();
        for (int i = 0; i < n; ++i) {
            auto region = dynamic_cast<Region *>(m_graph.GetVertexByIndex(i));
            if (region && HasAllFlags(region->GetFlags(), flags))
                region->SetColor(r, g, b);
        }
    }

    void Decomposition::SetRegionColorsBasedOnClearance(const Flags flags) {
        const int n = m_graph.GetNrVertices();
        for (int i = 0; i < n; ++i) {
            auto r = dynamic_cast<Region *>(m_graph.GetVertexByIndex(i));
            if (r && HasAllFlags(r->GetFlags(), flags)) {
                const double c = r->GetClearance() / m_maxRegionClearance;
                r->SetColor(c, c, c);
            }
        }
    }

    void Decomposition::SetRegionColorsBasedOnPathCost(const int goalIndex, const Flags flags) {
        auto colors = Colormap::GetSingleton();

        const int n = m_graph.GetNrVertices();
        for (int i = 0; i < n; ++i) {
            auto r = dynamic_cast<Region *>(m_graph.GetVertexByIndex(i));
            if (r && HasAllFlags(r->GetFlags(), flags)) {
                auto data = r->GetPathDataToGoals();
                if (data && data->size() > goalIndex) {
                    const double c = 1.0 - (*data)[goalIndex]->m_cost / m_maxPathCosts[goalIndex];
                    r->SetColor(colors->GetRed(c), colors->GetGreen(c), colors->GetBlue(c));
                }
            }
        }
    }

    void Decomposition::PathFromIdsToPoints(const int n, const Id rids[], std::vector<double> &pts) {
        double p[3];

        auto sim = GetSimulator();
        const int dim = sim->GetPositionAllocator()->GetDim();

        for (int i = 0; i < n; ++i) {
            auto r = dynamic_cast<Region *>(GetGraph()->GetVertex(rids[i]));
            if (r) {
                sim->GetPositionCfg(r->GetCfg(), p);
                for (int j = 0; j < dim; ++j)
                    pts.push_back(p[j]);
            }
        }
    }
}
