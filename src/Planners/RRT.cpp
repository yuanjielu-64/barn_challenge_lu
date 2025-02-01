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
#include "Planners/RRT.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include <iostream>

namespace Antipatrea {

    bool RRT::Solve(const int nrIters, const double tmax, bool &canBeSolved) {
        Timer::Clock clk;
        std::vector<double> pos;
        Group *group;

        Timer::Start(clk);

        if (m_tree.GetNrVertices() == 0) {
            if (Start() == false) {
                canBeSolved = false;
                return false;
            }

            if (m_tree.GetNrVertices() >= 1) {
                auto vRoot = dynamic_cast<MPTreeVertex *>(m_tree.GetVertex(0));
                auto keyRoot = dynamic_cast<GroupKeyTour *>(vRoot->GetGroup()->GetKey());
                auto ord = &(keyRoot->GetTour()->m_order);
                for (int i = 1; i < (int) ord->size(); ++i)
                    m_order.push_back(keyRoot->GetUnreachedGoalsKey()->GetUnreachedGoals()->operator[]((*ord)[i] - 1));

                Logger::m_out << "MAIN TOUR:";
                for (int i = 0; i < (int) m_order.size(); ++i)
                    Logger::m_out << m_order[i] << " ";
                Logger::m_out << std::endl;

                m_index = 0;
                if (m_order.size() > 0)
                    m_gidCurr = m_order[0];


            }
        }

        std::vector<double> aux;


        aux.resize(GetSimulator()->GetPositionAllocator()->GetDim());

        pos.resize(GetSimulator()->GetPositionAllocator()->GetDim());
        for (int i = 0; i < nrIters && Timer::Elapsed(clk) < tmax && !IsSolved(); ++i) {
            if (RandomUniformReal() <= 0.05 && m_index >= 0 && m_index < m_order.size())
                GetProblem()->GetGoals()->operator[](m_order[m_index])->SamplePointInside(&pos[0]);
            else
                GetSimulator()->SamplePosition(&pos[0]);

            double dmin = INFINITY;
            int best = -1;
            int a = m_use_vids.size();
            for (int j = m_use_vids.size() - 1; j >= 0 && j > (((int) m_use_vids.size()) - 5000); --j) {
                GetSimulator()->GetPositionState(
                        dynamic_cast<MPTreeVertex *>(m_tree.GetVertex(m_use_vids[j]))->GetState(), &aux[0]);
                const double d = GetSimulator()->DistancePositions(&aux[0], &pos[0]);
                if (d < dmin) {
                    dmin = d;
                    best = j;

                    if (RandomUniformReal() < 0.0005)
                        break;
                }
            }

            if (best >= 0) {
//		Logger::m_out << " extending from " << m_use_vids[best] << " out of " << m_use_vids.size() << std::endl;
                ExtendFrom(m_use_vids[best], &pos[0]);
            }

        }


        return IsSolved();
    }

    bool RRT::CompleteGroup(Group &group, MPTreeVertex &v) {
        Timer::Clock clk;
        Timer::Start(clk);

        auto tkey = dynamic_cast<GroupKeyTour *>(group.GetKey());

        if (tkey == NULL)
            return false;

        bool ok = false;
        auto tgen = GetTourGenerator();
        auto kreg = tkey->GetRegionKey();
        auto kgoals = tkey->GetUnreachedGoalsKey();
        auto unreached = kgoals->GetUnreachedGoals();
        auto ng = unreached->size();
        auto decomp = GetDecomposition();
        auto bounds = GetProblem()->GetTimeBounds();
        auto goals = GetProblem()->GetGoals();
        double cost;

        auto data = dynamic_cast<Region *>(decomp->GetGraph()->GetVertex(kreg->GetId()))->GetPathDataToGoals();

        tgen->SetNrSites(1 + ng);
        tgen->SetStartTime(tkey->GetTour()->GetStartTime());
        tgen->SetBounds(0, 0.0, INFINITY);
        tgen->SetDuration(0, 0, 0.0);
        for (int i = 0; i < ng; ++i) {
            tgen->SetDuration(i + 1, i + 1, 0.0);
            tgen->SetBounds(i + 1, (*bounds)[2 * (*unreached)[i]], (*bounds)[2 * (*unreached)[i] + 1]);
            cost = (*data)[(*unreached)[i]]->m_cost;
            tgen->SetDuration(0, i + 1, cost);
            tgen->SetDuration(i + 1, 0, cost);

            if (cost == INFINITY) {
                Logger::m_out << "infinite cost" << std::endl;
                Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

                return false;
            }


        }

        for (int i = 0; i < ng; ++i) {
            data = (*goals)[(*unreached)[i]]->GetPathDataToGoals();
            for (int j = i + 1; j < ng; ++j) {
                cost = (*data)[(*unreached)[j]]->m_cost;
                tgen->SetDuration(i + 1, j + 1, cost);
                tgen->SetDuration(j + 1, i + 1, cost);

                if (cost == INFINITY) {
                    Logger::m_out << "inifinite cost" << std::endl;
                    Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

                    return false;
                }
            }
        }

        // check if tour from parent can be used
        if (v.GetParent()) {
            auto parent = dynamic_cast<MPTreeVertex *>(v.GetParent());
            auto pkey = dynamic_cast<GroupKeyTour *>(parent->GetGroup()->GetKey());
            if (kgoals->SameContent(*(pkey->GetUnreachedGoalsKey()))) {
                tkey->GetTour()->m_order = pkey->GetTour()->m_order;
                ok = tgen->FromOrderToTimes(*(tkey->GetTour()));
                //   if (ok)
                //    Logger::m_out << "using parent order " << *(tkey->GetTour()) << std::endl;

            }
        }


        if (!ok) {
            ok = tgen->GenerateTour(*(tkey->GetTour()));
            Stats::GetSingleton()->AddValue("NrCallsGenerateTour", 1);
            if (m_shouldWait)
                Stats::GetSingleton()->AddValue("NrCallsGenerateTourWaiting", 1);
            //  Stats::GetSingleton()->AddValue("SanityCheck", tgen->SanityCheck() == false);

        }

        if (ok && v.GetParent() && v.GetReachedGoal() >= 0) {

            auto parent = dynamic_cast<MPTreeVertex *>(v.GetParent());
            auto pkey = dynamic_cast<GroupKeyTour *>(parent->GetGroup()->GetKey());

            Logger::m_out << "parent unreached goals:" << (*(pkey->GetUnreachedGoalsKey())) << std::endl
                          << "parent tour " << *(pkey->GetTour()) << std::endl
                          << "vertex unreached goals:" << *kgoals << std::endl
                          << "vertex reached goal: " << v.GetReachedGoal() << std::endl;

            auto ord = &(pkey->GetTour()->m_order);
            for (int i = 1; i < (int) ord->size(); ++i)
                Logger::m_out << pkey->GetUnreachedGoalsKey()->GetUnreachedGoals()->operator[]((*ord)[i] - 1) << " ";
            Logger::m_out << std::endl;

        }


        if (ok) {
            const double dur = tkey->GetTour()->GetEndTime() - tkey->GetTour()->GetStartTime();
            const int nrSites = tkey->GetTour()->GetNrSites();

            if (dur < Constants::EPSILON && nrSites > 1) {
                Logger::m_out << "unreached:";
                for (int i = 0; i < ng; ++i)
                    Logger::m_out << (*unreached)[i] << " ";
                Logger::m_out << std::endl;
                Logger::m_out << "goals to regions: ";
                for (int i = 0; i < (int) goals->size(); ++i)
                    Logger::m_out << "(" << i << "," << (*goals)[i]->GetKey() << ") ";
                Logger::m_out << std::endl;

                Logger::m_out << "what....... " << dur << "nrSites = " << nrSites << std::endl;
                Logger::m_out << *tkey << std::endl;
                Logger::m_out << "tour generator " << std::endl;
                Logger::m_out << *tgen << std::endl;

                group.SetWeight(Constants::GROUP_MAX_WEIGHT);
            } else
                group.SetWeight(10000000000.0 / (dur * pow(2, nrSites)));


            /////////
            /*
              auto xgid = (*unreached)[tkey->GetTour()->m_order[1] - 1];
              auto xgdata = (*data)[xgid];

              const int dimPos = GetSimulator()->GetPositionAllocator()->GetDim();
              group.m_guide.clear();
              RegularizePointsAlongPath(xgdata->m_pts.size() / dimPos, &xgdata->m_pts[0], m_follow.GetRadius(), dimPos, group.m_guide);
            */
            /////////g255
            Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

            return true;
        }

        Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

        return false;
    }

    void RRT::AddVertex(MPTreeVertex *const v) {

        MPTree::AddVertex(v);

        if (m_index >= 0 && m_index < m_order.size() && m_order[m_index] == v->GetReachedGoal()) {
            //goal reached
            m_use_vids.clear();

            ++m_index;
            if (m_index < m_order.size())
                m_gidCurr = m_order[m_index];
            else
                m_vidSolved = v->GetKey();

            Logger::m_out << "reached " << m_index << "/" << m_order.size() << " goals" << std::endl;
        }
        m_use_vids.push_back(v->GetKey());
        //Logger::m_out << " added to use_vids = " << m_use_vids.size() << std::endl;

    }
}

