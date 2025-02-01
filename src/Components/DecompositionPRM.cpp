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
#include "Components/DecompositionPRM.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"

namespace Antipatrea {

    void DecompositionPRM::Construct(const double tmax) {
        Timer::Clock clk;
        Timer::Clock t1;
        Timer::Clock t2;
        Timer::Start(clk);

        m_proximityAuxCfg.resize(GetSimulator()->GetCfgAllocator()->GetDim());
        GetSimulator()->GetCfg(&m_proximityAuxCfg[0]);

        auto keyInit = AddRegion(&m_proximityAuxCfg[0]);
        AddGoals();
        SetInitialAndGoalsPairs(keyInit);

        while (!AreInitialAndGoalsConnected() && GetGraph()->GetNrVertices() < GetMaxNrVertices()) {
            Timer::Start(t2);
            AddRegions(m_batchSize, tmax - Timer::Elapsed(clk));
            //Logger::m_out << "...nr regions " << GetGraph()->GetNrVertices() << std::endl;
            Stats::GetSingleton()->AddValue("TimeAddRegions", Timer::Elapsed(t2));

            Timer::Start(t1);
            ConnectRegions(tmax - Timer::Elapsed(clk));
            Stats::GetSingleton()->AddValue("TimeConnectRegions", Timer::Elapsed(t1));
        }
        const int nvs = GetGraph()->GetNrVertices();
        const int key = GetProblem()->GetGoals()->operator[](0)->GetKey();
        const int count = m_proximity.GetKeys()->size();
        m_proximity.Clear();

        if (AreInitialAndGoalsConnected() && GetProblem()->GetGoals()->size() > 0)
            for (int i = GetGraph()->GetNrVertices() - 1; i >= 0; --i)
                if (GetGraph()->AreVerticesPathConnected(GetGraph()->GetVertexByIndex(i)->GetKey(), key))
                    m_proximity.AddKey(GetGraph()->GetVertexByIndex(i)->GetKey());

        auto edges = GetGraph()->GetEdges();
        for (auto &iter: *edges) {
            auto edge = iter.second;
            auto idFrom = edge->GetFromVertexKey();
            auto idTo = edge->GetToVertexKey();
            if (!GetGraph()->AreVerticesPathConnected(idFrom, key) ||
                !GetGraph()->AreVerticesPathConnected(idTo, key))
                edge->SetCost(INFINITY);
        }


//        Logger::m_out << "to be connected " << m_regionsToBeConnected.size() << " vertices = "
//                      << GetGraph()->GetNrVertices() << std::endl
//                      << "nrComponents = " << GetGraph()->GetComponents()->GetNrComponents() << std::endl
//                      << "initial & goals connected = " << AreInitialAndGoalsConnected() << std::endl
//                      << "removed from proximity = " << (count - m_proximity.GetKeys()->size()) << " proximity size = "
//                      << m_proximity.GetKeys()->size() << std::endl
//                      << "keyInit = " << keyInit << std::endl;


        if (AreInitialAndGoalsConnected())
            PathsToGoals();

        Stats::GetSingleton()->AddValue("TimeConnectRegions", Timer::Elapsed(t1));
    }

    bool DecompositionPRM::Construct() {
        Timer::Clock clk;
        Timer::Clock t1;
        Timer::Clock t2;
        Timer::Start(clk);

        m_proximityAuxCfg.resize(GetSimulator()->GetCfgAllocator()->GetDim());
        GetSimulator()->GetCfg(&m_proximityAuxCfg[0]);

        auto keyInit = AddRegion(&m_proximityAuxCfg[0]);
        AddGoals();
        SetInitialAndGoalsPairs(keyInit);

        while (!AreInitialAndGoalsConnected() && GetGraph()->GetNrVertices() < GetMaxNrVertices()) {
            Timer::Start(t2);
            AddRegions(m_batchSize, 7);
            //Logger::m_out << "...nr regions " << GetGraph()->GetNrVertices() << std::endl;
            Stats::GetSingleton()->AddValue("TimeAddRegions", Timer::Elapsed(t2));

            Timer::Start(t1);
            ConnectRegions(7);
            Stats::GetSingleton()->AddValue("TimeConnectRegions", Timer::Elapsed(t1));
        }
        const int nvs = GetGraph()->GetNrVertices();
        const int key = GetProblem()->GetGoals()->operator[](0)->GetKey();
        const int count = m_proximity.GetKeys()->size();
        m_proximity.Clear();

        if (AreInitialAndGoalsConnected() && GetProblem()->GetGoals()->size() > 0)
            for (int i = GetGraph()->GetNrVertices() - 1; i >= 0; --i)
                if (GetGraph()->AreVerticesPathConnected(GetGraph()->GetVertexByIndex(i)->GetKey(), key))
                    m_proximity.AddKey(GetGraph()->GetVertexByIndex(i)->GetKey());

        auto edges = GetGraph()->GetEdges();
        for (auto &iter: *edges) {
            auto edge = iter.second;
            auto idFrom = edge->GetFromVertexKey();
            auto idTo = edge->GetToVertexKey();
            if (!GetGraph()->AreVerticesPathConnected(idFrom, key) ||
                !GetGraph()->AreVerticesPathConnected(idTo, key))
                edge->SetCost(INFINITY);
        }


//        Logger::m_out << "to be connected " << m_regionsToBeConnected.size() << " vertices = "
//                      << GetGraph()->GetNrVertices() << std::endl
//                      << "nrComponents = " << GetGraph()->GetComponents()->GetNrComponents() << std::endl
//                      << "initial & goals connected = " << AreInitialAndGoalsConnected() << std::endl
//                      << "removed from proximity = " << (count - m_proximity.GetKeys()->size()) << " proximity size = "
//                      << m_proximity.GetKeys()->size() << std::endl
//                      << "keyInit = " << keyInit << std::endl;


        if (AreInitialAndGoalsConnected())
            PathsToGoals();

        Stats::GetSingleton()->AddValue("TimeConnectRegions", Timer::Elapsed(t1));

        return true;
    }

    Id DecompositionPRM::LocateRegion(const double cfg[]) {
        m_proximityAuxCfg.resize(GetSimulator()->GetCfgAllocator()->GetDim());
        GetSimulator()->GetCfgAllocator()->Copy(&m_proximityAuxCfg[0], cfg);

        ProximityQuery<Id> query;

        query.SetKey(Constants::ID_UNDEFINED);
        return m_proximity.Neighbor(query);
    }

    void DecompositionPRM::AddGoals(void) {
        Decomposition::AddGoals();
        for (auto &goal: *(GetProblem()->GetGoals())) {
            m_proximity.AddKey(goal->GetKey());
            m_regionsToBeConnected.insert(goal->GetKey());
        }
    }

    Id DecompositionPRM::AddRegion(const double cfg[]) {
        auto r = new Region();
        double p[3];

        r->SetKey(GetGraph()->GetNrVertices());
        r->SetCfg(GetSimulator()->GetCfgAllocator()->Copy(cfg));

        GetSimulator()->GetPositionCfg(cfg, p);
        r->SetClearance(GetSimulator()->ClearancePosition(p));
        UpdateMinMaxRegionClearances(r->GetClearance());

        GetGraph()->AddVertex(r);
        m_proximity.AddKey(r->GetKey());
        m_regionsToBeConnected.insert(r->GetKey());

        return r->GetKey();
    }

    int DecompositionPRM::AddRegions(const int nrCfgs, const double tmax) {
        auto sim = GetSimulator();
        auto cfg = sim->GetCfgAllocator()->New();
        int count = 0;
        Timer::Clock clk;

        Timer::Start(clk);
        while (count < nrCfgs && Timer::Elapsed(clk) < tmax && GetGraph()->GetNrVertices() < GetMaxNrVertices()) {
            sim->SampleCfg(cfg);
            if (sim->IsValidCfg(cfg)) {
                ++count;
                AddRegion(cfg);
                cfg = sim->GetCfgAllocator()->New();
            }
        }
        sim->GetCfgAllocator()->Delete(cfg);

        return count;
    }

    int DecompositionPRM::ConnectRegions(const double tmax) {
        Timer::Clock clk;
        Timer::Start(clk);

        while ((!AreInitialAndGoalsConnected() || !GetStopWhenConnected()) &&
               m_regionsToBeConnected.empty() == false)
            ConnectRegion(*(m_regionsToBeConnected.begin()));

        return m_regionsToBeConnected.size();
    }

    void DecompositionPRM::ConnectRegion(const Id key) {
        ProximityQuery<Id> query;
        ProximityResults<Id> res;

        query.SetNrNeighbors(GetNrNeighbors());
        query.SetKey(key);
        m_proximity.Neighbors(query, res);

        const int n = res.GetNrResults();
        for (int i = 0; i < n; ++i)
            GenerateEdge(key, res.GetKey(i), res.GetDistance(i));
        m_regionsToBeConnected.erase(key);
    }

    bool DecompositionPRM::GenerateEdge(const Id key1, const Id key2, const double d) {
        auto sim = GetSimulator();

        if (key1 == key2 ||
            (RandomUniformReal() > GetProbabilityAllowCycles() && GetGraph()->AreVerticesPathConnected(key1, key2)) ||
            m_attempts.find(std::make_pair(key1, key2)) != m_attempts.end())
            return false;

        m_attempts.insert(std::make_pair(key1, key2));
        m_attempts.insert(std::make_pair(key2, key1));

        auto r1 = dynamic_cast<Region *>(GetGraph()->GetVertex(key1));
        auto r2 = dynamic_cast<Region *>(GetGraph()->GetVertex(key2));

        if (d > GetOneStepDistance()) {
            auto dt = AdjustTimeStep(GetOneStepDistance() / d);

            if (sim->IsValidPathCfgs(r1->GetCfg(), r2->GetCfg(), dt, dt, 1.0 - dt) == false)
                return false;
        }

        DecompositionEdge *e1 = new DecompositionEdge();
        e1->SetFromToVertexKeys(key1, key2);
        e1->SetValues(*sim, *r1, *r2);
        GetGraph()->AddEdge(e1);

        DecompositionEdge *e2 = new DecompositionEdge();
        e2->SetFromToVertexKeys(key2, key1);
        e2->CopyValuesFrom(*e1);
        GetGraph()->AddEdge(e2);

        return true;
    }

    double DecompositionPRM::ProximityDistanceFn(const Id key1, const Id key2, DecompositionPRM *prm) {
        auto v1 = prm->GetGraph()->GetVertex(key1);
        auto v2 = prm->GetGraph()->GetVertex(key2);
        const double *cfg1 = v1 ? dynamic_cast<Region *>(v1)->GetCfg() : &(prm->m_proximityAuxCfg[0]);
        const double *cfg2 = v2 ? dynamic_cast<Region *>(v2)->GetCfg() : &(prm->m_proximityAuxCfg[0]);

        return prm->GetSimulator()->DistanceCfgs(cfg1, cfg2);
    }
}
