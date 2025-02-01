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
#include "Components/DecompositionGeometric.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"

namespace Antipatrea {

    void DecompositionGeometric::Construct(const double tmax) {

        AddRegions();
        ConnectRegions();

        AddGoals();
        ConnectGoals();
        PathsToGoals();

        std::vector<double> cfg;
        cfg.resize(GetSimulator()->GetCfgAllocator()->GetDim());
        GetSimulator()->GetCfg(&cfg[0]);

        SetInitialAndGoalsPairs(LocateRegion(&cfg[0]));

    }

    bool DecompositionGeometric::Construct() {
        Timer::Clock time;
        Timer::Start(time);

        AddRegions();
        Logger::m_out << "txx1 " << Timer::Elapsed(time) << std::endl;
        ConnectRegions();
        Logger::m_out << "txx2 " << Timer::Elapsed(time) << std::endl;

        AddGoals();
        Logger::m_out << "txx3 " << Timer::Elapsed(time) << std::endl;
        ConnectGoals();
        Logger::m_out << "txx4 " << Timer::Elapsed(time) << std::endl;
        PathsToGoals();
        Logger::m_out << "txx5 " << Timer::Elapsed(time) << std::endl;

        std::vector<double> cfg;
        cfg.resize(GetSimulator()->GetCfgAllocator()->GetDim());
        GetSimulator()->GetCfg(&cfg[0]);

        SetInitialAndGoalsPairs(LocateRegion(&cfg[0]));
        Logger::m_out << "txx6 " << Timer::Elapsed(time) << std::endl;

        return true;
    }


    void DecompositionGeometric::ConnectGoals(void) {
        const double dtol = Constants::EPSILON;

        RegionGeometric *r;
        RegionGeometric *rother;
        auto goals = GetProblem()->GetGoals();
        const int ng = goals->size();
        const int nv = m_graph.GetNrVertices();
        for (int i = 0; i < ng; ++i) {
            r = (*goals)[i];
            for (int j = 0; j < nv; ++j) {
                rother = dynamic_cast<RegionGeometric *>(m_graph.GetVertexByIndex(j));
                if (r != NULL && r != rother && r->IsNeighbor(*rother, dtol)) {
                    DecompositionEdge *e1 = new DecompositionEdge();
                    e1->SetFromToVertexKeys(r->GetKey(), rother->GetKey());
                    e1->SetValues(*GetSimulator(), *r, *rother);
                    e1->SetFlags(DecompositionEdge::STATUS_GOAL);
                    m_graph.AddEdge(e1);

                    DecompositionEdge *e2 = new DecompositionEdge();
                    e2->SetFromToVertexKeys(rother->GetKey(), r->GetKey());
                    e2->SetFlags(DecompositionEdge::STATUS_GOAL);
                    e2->CopyValuesFrom(*e1);
                    m_graph.AddEdge(e2);

                    if (e1->GetCost() < Constants::EPSILON)
                        Logger::m_out << "............................edge " << r->GetKey() << " " << rother->GetKey()
                                      << " has cost " << e1->GetCost() << std::endl;

                }
            }
        }
    }

    void DecompositionGeometric::DrawRegionsShape(const Flags flags) {
        const int n = m_graph.GetNrVertices();
        for (int i = 0; i < n; ++i) {
            auto r = dynamic_cast<RegionGeometric *>(m_graph.GetVertexByIndex(i));
            if (r && HasAllFlags(r->GetFlags(), flags)) {
                GDrawColor(r->GetColor());
                r->DrawShape();
            }
        }
    }
}
