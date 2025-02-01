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
#include "Planners/Juve.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include "Utils/GDraw.hpp"
#include <iostream>

namespace Antipatrea {

    bool Juve::Solve(const int nrIters, const double tmax, bool &canBeSolved) {
        m_groupFollowOriginator = NULL;

        Timer::Clock clk;
        std::vector<double> pos;
        Group *group;

        m_follow.SetDim(GetSimulator()->GetPositionAllocator()->GetDim());

        Timer::Start(clk);

        if (m_tree.GetNrVertices() == 0) {
            Logger::m_out << "JUVE" << std::endl;

            if (Start() == false) {
                canBeSolved = false;
                return false;
            }

        }

        int oldNrGroups = m_groups.GetNrGroups();
        int counter = 0;
        bool useFollow = true;

        pos.resize(GetSimulator()->GetPositionAllocator()->GetDim());
//        && Timer::Elapsed(clk) < tmax
        for (int i = 0; i < nrIters && !IsSolved(); ++i) {
            if (m_shouldWait)
                ExtendWait();
            else if (useFollow) {
                ++m_followProgress.m_nrIters;

                GetGroupSelector()->SetGroups(&m_followGroups);
                group = GetGroupSelector()->SelectGroup();

                m_follow.Sample(dynamic_cast<GroupKeyId *>(group->GetKey())->GetId(), &pos[0]);

                const int vid = SelectVertex(*group, &pos[0]);
                auto v = dynamic_cast<MPTreeVertex *>(m_tree.GetVertex(vid));
                // m_extendStopAtTarget = v->GetNextWaypt() == m_follow.GetNrPoints() - 1;
                ExtendFrom(vid, &pos[0]);

                m_followGroups.UpdateWeightGroup(*group, group->GetWeight() * GetDiscountSelect());

                //const int insane =      (int) Stats::GetSingleton()->GetValue("SanityCheck");
                //if(insane > 0)
                // {
                //
//	  m_followGroups.UpdateWeightGroup(*group, group->GetWeight() * GetDiscountSelect());
//	  if(m_groupFollowOriginator != NULL)
//	      m_groupFollowOriginator->SetWeight(m_groupFollowOriginator->GetWeight() * GetDiscountSelect());
//	  
//	      }


                if (m_shouldWait)
                    continue;

                ++m_followProgress.m_nrIters;
                FollowUpdateIfNoProgress();

                if (m_groups.GetNrGroups() == oldNrGroups)
                    ++counter;
                if (counter >= GetCounterFromFollowToExplore() && RandomUniformReal() > (1.0 / counter)) {
                    useFollow = false;
                    counter = 0;
                    //Logger::m_out << "switching to exploration " << m_groups.GetNrGroups() << std::endl;
                }
            } else {
                GetGroupSelector()->SetGroups(&m_groups);
                group = GetGroupSelector()->SelectGroup();

                /*
                const int dimPos = GetSimulator()->GetPositionAllocator()->GetDim();

                if(RandomUniformReal() > 0.05 && group->m_guide.size() > dimPos)
                {
                const int n = (int) (group->m_guide.size() / dimPos);
                const int which = RandomUniformInteger(1, std::min(4, n - 1));
                const double *target = &group->m_guide[which * dimPos];
                RandomPointInsideSphere(dimPos, target, m_follow.GetRadius(), &pos[0]);
                }
                else*/
                GetSimulator()->SamplePosition(&pos[0]);

                ExtendFrom(SelectVertex(*group, &pos[0]), &pos[0]);

                m_groups.UpdateWeightGroup(*group, group->GetWeight() * GetDiscountSelect());

                ++counter;
                if (counter >= GetCounterFromExploreToFollow() && RandomUniformReal() > (1.0 / counter)) {
                    useFollow = true;
                    counter = 0;
                    oldNrGroups = m_groups.GetNrGroups();
                    //   Logger::m_out << "switching to follow " << m_groups.GetNrGroups() << std::endl;
                }
            }
        }

        return IsSolved();
    }

    bool Juve::CompleteGroup(Group &group, MPTreeVertex &v) {
        Timer::Clock clk;
        Timer::Start(clk);

        auto tkey = dynamic_cast<GroupKeyTour *>(group.GetKey());

        if (tkey == NULL)
            return false;

        auto kreg = tkey->GetRegionKey();

        auto goals = GetProblem()->GetGoals();

        auto data = dynamic_cast<Region *>(GetDecomposition()->GetGraph()->GetVertex(kreg->GetId()))->GetPathDataToGoals();
        const double dur = Algebra::PointDistance(2, tkey->GetState(), (*goals)[0]->GetCfg());

        //const double dur = (*data)[0]->m_cost;
        group.SetWeight(10000000000.0 / dur);

        Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

        return true;

    }

    bool Juve::SetupVertex(MPTreeVertex &v) {
        if (MPTree::SetupVertex(v) == false)
            return false;
        if (m_tree.GetNrVertices() == 0)
            return true;

        std::vector<double> pos;
        pos.resize(GetSimulator()->GetPositionAllocator()->GetDim());
        GetSimulator()->GetPositionState(v.GetState(), &pos[0]);

        v.SetNextWaypt(v.GetParent() ? dynamic_cast<MPTreeVertex *>(v.GetParent())->GetNextWaypt() : 0);
        if (m_follow.Reached(v.GetNextWaypt(), &pos[0]))
            v.SetNextWaypt(v.GetNextWaypt() + 1);
        else if (m_follow.IsInside(v.GetNextWaypt(), &pos[0]) == false) {
            //  m_extendStopAtTarget = v.GetNextWaypt() == m_follow.GetNrPoints() - 1;
            return false;
        }
        // m_extendStopAtTarget = v.GetNextWaypt() == m_follow.GetNrPoints() - 1;

        return true;
    }

    void Juve::AddVertex(MPTreeVertex *const v) {
        MPTree::AddVertex(v);

        if (m_follow.GetNrPoints() == 0)
            FollowGenerateWaypts();

        FollowUpdateGroups(*v);
    }

    void Juve::FollowUpdateGroups(MPTreeVertex &v) {
        GroupKeyId *key = new GroupKeyId();

        key->SetId(v.GetNextWaypt());

        auto g = m_followGroups.FindGroup(*key);

        if (g == NULL) {
            g = new Group();
            g->SetKey(key);
            g->SetWeight(m_follow.Weight(v.GetNextWaypt()));
            m_followGroups.AddGroup(g);
        } else
            delete key;
        g->GetVertices()->push_back(v.GetKey());

        if (v.GetNextWaypt() >= m_follow.GetNrPoints())
            m_followSolved = true;
    }

    void Juve::FollowClear(void) {
        m_followGroups.Clear();
        m_follow.Clear();
        m_followSolved = false;
        m_followProgress.m_nrIters = 0;
        m_followProgress.m_nrGroups = 0;
    }

    void Juve::FollowUpdateIfNoProgress(void) {
        if (m_followSolved) {
            FollowGenerateWaypts();
            return;
        }

        if (m_shouldWait)
            return;

        const double a = 1.0 / (m_followProgress.m_nrItersEnd - m_followProgress.m_nrItersStart);
        const double b = -m_followProgress.m_nrItersStart * a;

        if (m_followProgress.m_nrIters > m_followProgress.m_nrItersStart) {
            if (m_followProgress.m_nrGroups >= m_followGroups.GetNrGroups()) {
                // no progress, select new follow
                if (RandomUniformReal() < 0.5 * pow(2, a * m_followProgress.m_nrIters + b))
                    FollowGenerateWaypts();
            } else {
                m_followProgress.m_nrIters = 0;
                m_followProgress.m_nrGroups = m_followGroups.GetNrGroups();
            }
        }
    }

    void Juve::AllPtsInTour(GroupKeyTour &tkey) {
        m_gTourPts.clear();

        GetProblem()->m_gShowIds.resize(GetProblem()->GetGoals()->size());
        for (int i = 0; i < GetProblem()->m_gShowIds.size(); ++i)
            GetProblem()->m_gShowIds[i] = -1;

        return;


        auto tour = tkey.GetTour();
        auto kgoals = tkey.GetUnreachedGoalsKey();
        auto unreached = kgoals->GetUnreachedGoals();
        auto rid = tkey.GetRegionKey()->GetId();


        if (tour->m_order.size() < 2 || unreached->size() == 0 || rid < 0)
            return;

        auto gid = (*unreached)[tour->m_order[1] - 1];
        auto data = dynamic_cast<Region *>(GetDecomposition()->GetGraph()->GetVertex(rid))->GetPathDataToGoals();
        auto gdata = (*data)[gid];

        m_gTourPts.insert(m_gTourPts.end(), gdata->m_pts.begin(), gdata->m_pts.end());

        Logger::m_out << "TOUR goals = " << gid << " ";

        GetProblem()->m_gShowIds[gid] = 0;

        for (int i = 2; i < tour->m_order.size(); ++i) {
            auto gidPrev = (*unreached)[tour->m_order[i - 1] - 1];
            auto gidCurr = (*unreached)[tour->m_order[i] - 1];

            GetProblem()->m_gShowIds[gidCurr] = i - 1;

            Logger::m_out << gidCurr << " ";
            data = GetProblem()->GetGoals()->operator[](gidPrev)->GetPathDataToGoals();
            gdata = (*data)[gidCurr];
            //   m_gTourPts.insert(m_gTourPts.end(), gdata->m_pts.begin(), gdata->m_pts.end());
        }
        Logger::m_out << std::endl;

    }


    void Juve::FollowGenerateWaypts(void) {
        static int count = 0;
        // Logger::m_out << "new lead........" << ++count << " nrGroups = " << m_groups.GetNrGroups() << std::endl;

        FollowClear();
        GetGroupSelector()->SetGroups(&m_groups);

        auto group = m_groupFollowOriginator = GetGroupSelector()->SelectGroup();
        auto tkey = dynamic_cast<GroupKeyTour *>(group->GetKey());
        auto kgoals = tkey->GetUnreachedGoalsKey();
        auto unreached = kgoals->GetUnreachedGoals();
        auto kreg = tkey->GetRegionKey();
        auto rid = kreg->GetId();
        auto tour = tkey->GetTour();

        group->SetWeight(group->GetWeight() * GetDiscountSelect());

        if (tour->m_order.size() < 2 || unreached->size() == 0 || rid < 0)
            return;

        auto gid = m_gidCurr = (*unreached)[tour->m_order[1] - 1];
        auto data = dynamic_cast<Region *>(GetDecomposition()->GetGraph()->GetVertex(rid))->GetPathDataToGoals();
        auto gdata = (*data)[gid];

        std::vector<double> pts;

        const int dimPos = GetSimulator()->GetPositionAllocator()->GetDim();
        RegularizePointsAlongPath(gdata->m_pts.size() / dimPos, &gdata->m_pts[0], m_follow.GetRadius(), dimPos, pts);
        for (int j = 0; j < (int) pts.size(); j += dimPos)
            m_follow.AddPoint(&pts[j]);

        const int vid = SelectVertex(*group);
        auto v = dynamic_cast<MPTreeVertex *>(m_tree.GetVertex(vid));
        auto g = new Group();
        auto gkey = new GroupKeyId();

        v->SetNextWaypt(0);
        gkey->SetId(v->GetNextWaypt());
        g->SetKey(gkey);
        g->GetVertices()->push_back(vid);
        g->SetWeight(m_follow.Weight(v->GetNextWaypt()));
        m_followGroups.AddGroup(g);

        // Logger::m_out << "follow has " << m_follow.GetNrPoints() << " toward " << gid << std::endl;

        AllPtsInTour(*tkey);

    }

    void Juve::Draw(void) const {
        MPTree::Draw();
        //m_follow.Draw();
        GDrawColor(0.2, 0.2, 0.2);
        GDrawLineWidth(4.0);

        for (int i = 2; i < m_gTourPts.size(); i += 2)
            GDrawSegment2D(&m_gTourPts[i - 2], &m_gTourPts[i]);

    }
}
