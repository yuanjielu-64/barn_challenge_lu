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
#include "Planners/MPTree.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include <iostream>

namespace Antipatrea {

    void MPTree::InitializeGroups(void) {
        auto group = NewGroup();
        group->SetKey(NewGroupKey());
        group->GetKey()->SetContent(*GetProblem());
        m_groups.AddGroup(group);
    }

    bool MPTree::Start(void) {

        m_nrGroupsWhileWaiting = 0;

        if (!GetSimulator()->IsValidState()) {
            Logger::m_out << "error MotionTreePlanner::Start ... initial state is in collision" << std::endl;
            return false;
        }

        auto v = NewVertex();
        if (!SetupVertex(*v)) {
            Logger::m_out << "error MotionTreePlanner::Start ... initial state could not be added" << std::endl;
            delete v;
            return false;
        }

        InitializeGroups();
        v->SetGroup(m_groups.GetGroup(0));
        UpdateGroups(*v);

        if (!CompleteGroup(*(m_groups.GetGroup(0)), *v)) {
            Logger::m_out << "...problem cannot be solved" << std::endl;
            return false;
        }

        AddVertex(v);

        return true;
    }

    bool MPTree::GetSolution(Solution &sol) {
        if (m_vidSolved < 0)
            return false;

        sol.Clear();

        auto vids = sol.GetVertexSequence();
        auto states = sol.GetStateSequence();
        auto times = sol.GetTimeSequence();
        auto angle = sol.GetAngleSequence();
        auto velocity = sol.GetVelocitySequence();
        auto sim = GetSimulator();

        sol.SetCost(m_tree.GetPathFromRootToVertex(m_vidSolved, *vids));
        for (auto &vid: *vids) {
            auto v = dynamic_cast<MPTreeVertex *>(m_tree.GetVertex(vid));
            states->push_back(sim->GetStateAllocator()->Copy(v->GetState()));
            times->push_back(v->GetTime());
            angle->push_back(v->GetAngleVelocity());
            velocity->push_back(v->GetVelocity());
        }

        return true;
    }

    bool MPTree::UpdateGroups(MPTreeVertex &v) {
        if (m_enforceOrder && v.GetParent())
            return true;

        if (v.GetGroup())
            v.GetGroup()->GetKey()->UpdateContent(v);
        else {
            auto pkey = dynamic_cast<MPTreeVertex *>(v.GetParent())->GetGroup()->GetKey();
            auto key = NewGroupKey();
            key->SetContent(*pkey);
            key->UpdateContent(v);

            auto group = m_groups.FindGroup(*key);
            if (group == NULL) {
                group = NewGroup();
                group->SetKey(key);
                if (!CompleteGroup(*group, v)) {
                    delete group;
                    return false;
                }
                m_groups.AddGroup(group);
                if (m_shouldWait)
                    ++m_nrGroupsWhileWaiting;

            } else
                delete key;
            v.SetGroup(group);
        }
        v.GetGroup()->GetVertices()->push_back(v.GetKey());

        if (v.GetGroup() && v.GetGroup()->GetKey() && v.GetGroup()->GetKey()->IsSolved()) {
            m_vidSolved = v.GetKey();
            for (int &m_gShowId: GetProblem()->m_gShowIds)
                m_gShowId = -1;

        }

        return true;
    }

    bool MPTree::SetupVertex(MPTreeVertex &v) {

        std::vector<double> aux;
        v.SetKey(m_tree.GetNrVertices());
        v.SetState(GetSimulator()->GetStateAllocator()->New());

        GetSimulator()->GetState(v.GetState());

        if (GetDecomposition()) {
            aux.resize(GetSimulator()->GetCfgAllocator()->GetDim());
            GetSimulator()->GetCfgState(v.GetState(), &aux[0]);
            v.SetRegion(GetDecomposition()->LocateRegion(&aux[0]));
            v.SetVelocity(v.GetState()[3]);
            v.SetAngleVelocity(v.GetState()[4]);
//            v.path_cost_.push_back(0);
//            v.speed_cost_.push_back(2);
//            v.obs_cost_.push_back(3);
//            v.to_goal_cost_.push_back(10);

            if (v.GetRegion() < 0)
                return false;
        }

        if (v.GetParent())
            v.SetEdgeCost(GetSimulator()->DistanceStates(v.GetState(),
                                                         dynamic_cast<MPTreeVertex *>(v.GetParent())->GetState()));

        // if (v.GetParent())
        {
            // auto parent = dynamic_cast<MPTreeVertex *>(v.GetParent());
            // v.SetTime(parent->GetTime() + GetSimulator()->DistanceStates(v.GetState(), parent->GetState()));
        }

        aux.resize(GetSimulator()->GetPositionAllocator()->GetDim());
        GetSimulator()->GetPositionState(v.GetState(), &aux[0]);

        const int gid = GetProblem()->LocateGoal(&aux[0]);
        if (gid >= 0) {
            if (m_enforceOrder && m_gidCurr != gid)
                v.SetReachedGoal(Constants::ID_UNDEFINED);
            else if (GetProblem()->IsGoalReachedInTime(gid, v.GetTime())) {
                v.SetReachedGoal(gid);
                //v.SetReward((*GetProblem()->GetGoals())[gid]->GetReward());
                if (gid == m_gidCurr) {
                    //if (m_shouldWait)
                    //Logger::m_out << "done waiting for " << gid << std::endl;
                    m_shouldWait = false;
                }
                // Logger::m_out << " goal " << gid << " greached in time " << v.GetTime() << std::endl;
            } else {
                if (!m_shouldWait) {
                    //no need to wait in this setting
                    // m_shouldWait = gid == m_gidCurr;
                    //if (m_shouldWait)
                    //  Logger::m_out << "waiting for " << gid << std::endl;
                }
                v.SetReachedGoal(Constants::ID_UNDEFINED);
                // Logger::m_out << " goal " << gid << " greached earlier " << v.GetTime() << std::endl;
            }
        } else
            v.SetReachedGoal(Constants::ID_UNDEFINED); // GetProblem()->LocateGoal(&aux[0], v.GetTime()));

        return true;
    }

    void MPTree::ExtendWait() {
        if (m_gidCurr < 0)
            return;
        // m_extendStopAtTarget = true;
        std::vector<double> pos;
        pos.resize(GetSimulator()->GetPositionAllocator()->GetDim());
        GetProblem()->GetGoals()->operator[](m_gidCurr)->GetRepresentativePoint(&pos[0]);

        int count = 0;
        const int n = m_tree.GetNrVertices();
        int vid = n - 1;
        while (m_shouldWait && count < GetWaitMaxFailures()) {
            if (ExtendFrom(vid, &pos[0]) == EXTEND_COLLISION) {
                vid = RandomUniformInteger(std::max(n - 1, m_tree.GetNrVertices() - 10), m_tree.GetNrVertices() - 1);
                GetProblem()->GetGoals()->operator[](m_gidCurr)->SamplePointInside(&pos[0]);
                ++count;
            } else
                vid = m_tree.GetNrVertices() - 1;
        }
        m_shouldWait = false;
    }

    void MPTree::AddVertex(MPTreeVertex *const v) {
        m_tree.AddVertex(v);
    }

    MPTree::ExtendStatus MPTree::ExtendFrom(const Id vid, const double p[]) {
        Timer::Clock clk;
        auto sim = GetSimulator();
        auto parent = dynamic_cast<MPTreeVertex *>(m_tree.GetVertex(vid));
        sim->SetState(parent->GetState());

        const int nrSteps = (int) RandomUniformInteger(GetExtendMinNrSteps(), GetExtendMaxNrSteps());

        sim->StartSteerToPosition(p);
        std::vector<double> tmpCfg;
        tmpCfg.resize(GetSimulator()->GetCfgAllocator()->GetDim());
        for (int i = 0; i < nrSteps && !IsSolved(); ++i) {
            sim->SteerToPosition(p, m_shouldWait || m_extendStopAtTarget);

            Timer::Start(clk);
            const double dt = sim->SimulateOneStep();
            sim->GetCfg(&tmpCfg[0]);

            auto currs = GetSimulator()->GetStateAllocator()->New();
            sim->GetState(currs);

            Stats::GetSingleton()->AddValue("TimeSimulate", Timer::Elapsed(clk));

            Timer::Start(clk);
            if (!sim->IsValidState()) {
                Stats::GetSingleton()->AddValue("TimeCollision", Timer::Elapsed(clk));
                return EXTEND_COLLISION;
            }

            Stats::GetSingleton()->AddValue("TimeCollision", Timer::Elapsed(clk));

            auto vnew = NewVertex();
            vnew->SetParent(parent);
            vnew->SetTime(parent->GetTime() + dt);
            vnew->SetAngleVelocity(currs[2]);
            vnew->SetVelocity(currs[3]);
            vnew->SetDistance(parent->GetDistance() + sim->DistancePositions(parent->GetState(), &tmpCfg[0]));

            if (!SetupVertex(*vnew) || !UpdateGroups(*vnew)) {
                delete vnew;
                return EXTEND_COLLISION;
            }

            AddVertex(vnew);

            if (sim->HasReachedSteerPosition(p))
                return EXTEND_TARGET;
            parent = vnew;
        }

        return EXTEND_NORMAL;

    }

    Id MPTree::SelectVertex(const int n, const Id vids[], const double target[]) const {
        //return vids[RandomUniformInteger(0, n - 1)];

        auto sim = GetSimulator();
        double d;
        std::vector<double> weights;
        std::vector<double> pos;
        double sweights = 0.0;

        weights.resize(n);
        pos.resize(sim->GetPositionAllocator()->GetDim());
        for (int i = 0; i < n; ++i) {
            auto v = dynamic_cast<const MPTreeVertex *>(m_tree.GetVertex(vids[i]));
            if (target) {
                sim->GetPositionState(v->GetState(), &pos[0]);
                d = sim->DistancePositions(target, &pos[0]);
            } else
                d = 0.0;

            weights[i] = 1.0 / (v->GetTime() + d / std::max(sim->GetVelocityScaleMin() * sim->GetMaxVelocity(),
                                                            sim->GetVelocityState(v->GetState())));

            sweights += weights[i];
        }

        const double coin = RandomUniformReal(0.0, sweights);
        double w = 0.0;

        for (int i = 0; i < n; ++i)
            if ((w += weights[i]) >= coin)
                return vids[i];
        return vids[n - 1];
    }

    void MPTree::Draw(void) const {
        auto sim = GetSimulator();
        std::vector<double> p1;
        std::vector<double> p2;

        p1.resize(sim->GetPositionAllocator()->GetDim());
        p2.resize(sim->GetPositionAllocator()->GetDim());

        for (int i = m_tree.GetNrVertices() - 1; i >= 0; --i) {
            auto v = dynamic_cast<const MPTreeVertex *>(m_tree.GetVertexByIndex(i));
            if (v && v->GetParent()) {
                sim->GetPositionState(dynamic_cast<const MPTreeVertex *>(v->GetParent())->GetState(), &p1[0]);
                sim->GetPositionState(v->GetState(), &p2[0]);
                sim->DrawSegment(&p1[0], &p2[0]);
            }
        }

        for (int i = m_tree.GetNrVertices() - 1; i >= 0; --i) {
            auto v = dynamic_cast<const MPTreeVertex *>(m_tree.GetVertexByIndex(i));
            sim->GetPositionState(v->GetState(), &p1[0]);
            sim->DrawPosition(&p1[0]);
        }
    }
}
