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
#include "Programs/GManagerMP.hpp"
#include "Utils/GManager.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include <cstring>
#include <cstdlib>

namespace Antipatrea {
    void GManagerMP::HandleEventOnDisplay(void) {
        GManagerComponent::HandleEventOnDisplay();

        auto items = GetManager()->GetMenuItems();
        auto mp = GetSetup()->GetMP();
        auto sim = GetSetup()->GetSimulator();
        auto seq = m_solution.GetStateSequence();
        auto vids = m_solution.GetVertexSequence();
        auto times = m_solution.GetTimeSequence();
        std::string msg;

        std::vector<double> pos;
        std::vector<double> pos2;

        if (sim == NULL || mp == NULL)
            return;

        pos.resize(sim->GetPositionAllocator()->GetDim());
        pos2.resize(sim->GetPositionAllocator()->GetDim());


        char what[100];

        if (HasAllFlags((*items)[MENU_DRAW_SOLUTION_ANIMATION]->GetFlags(), GMenuItem::FLAG_ON) &&
            m_solutionIndex >= 0 && m_solutionIndex < seq->size()) {

            sim->GetPositionState((*seq)[m_solutionIndex], &pos[0]);
            sprintf(what, "%.1f", (*times)[m_solutionIndex]);


            const int goalReached = GetSetup()->GetProblem()->LocateGoal(&pos[0], (*times)[m_solutionIndex]);
            if (goalReached >= 0)
                GetSetup()->GetProblem()->m_gShowIds[goalReached] = -2;

            sim->SetState((*seq)[m_solutionIndex]);
            //   sim->DrawState();

            GDrawColor(1, 0.0, 0.0);

            GDrawString3D(what, pos[0], pos[1], pos[2] + 1.0, true);
        }

        if (HasAllFlags((*items)[MENU_DRAW_SOLUTION_TRAJECTORY]->GetFlags(), GMenuItem::FLAG_ON)) {
            for (int i = seq->size() - 1; i >= 1; --i) {
                sim->GetPositionState((*seq)[i - 1], &pos[0]);
                sim->GetPositionState((*seq)[i], &pos2[0]);
                sim->DrawSegment(&pos[0], &pos2[0]);
            }

            for (auto &s: *seq) {
                sim->GetPositionState(s, &pos[0]);
                sim->DrawPosition(&pos[0]);
            }
        }

        if (HasAllFlags((*items)[MENU_DRAW_SOLUTION_SAVED_STATES]->GetFlags(), GMenuItem::FLAG_ON)) {
            for (auto &curr: m_solutionSavedStates)
                if (curr >= 0 && curr < seq->size()) {
                    sim->SetState((*seq)[curr]);
                    sim->DrawState();

                }
        }

        GDrawColor(0.0, 1.0, 0.0);

        if (HasAllFlags((*items)[MENU_DRAW_TREE]->GetFlags(), GMenuItem::FLAG_ON))
            mp->Draw();

    }

    void GManagerMP::HandleEventOnTimer(void) {
        GManagerComponent::HandleEventOnTimer();

        auto items = GetManager()->GetMenuItems();

        if (HasAllFlags((*items)[MENU_DRAW_SOLUTION_ANIMATION]->GetFlags(), GMenuItem::FLAG_ON)) {
            if (m_solutionIndex < m_solution.GetStateSequence()->size())
                ++m_solutionIndex;
        }

        if (HasAllFlags((*items)[MENU_SOLVE_REPEAT]->GetFlags(), GMenuItem::FLAG_ON))
            OnSolve();
    }

    bool GManagerMP::HandleEventOnMenu(const int item) {
        if (item == MENU_SOLVE_ONCE) {
            OnSolve();
            return true;
        } else
            return GManagerComponent::HandleEventOnMenu(item);
    }

    bool GManagerMP::HandleEventOnNormalKeyPress(const int key) {
        auto items = GetManager()->GetMenuItems();
        auto seq = m_solution.GetStateSequence();

        if (key == '+' && m_solutionIndex < seq->size()) {
            ++m_solutionIndex;
            return true;
        } else if (key == '-' && m_solutionIndex > 0) {
            --m_solutionIndex;
            return true;
        } else if (key == 'z') {
            m_solutionIndex = 0;
            if (m_solutionIndex < seq->size())
                GetSetup()->GetSimulator()->SetState((*seq)[m_solutionIndex]);

            return true;
        } else if (key == '1')
            GManagerComponent::HandleEventOnMenu(MENU_SOLVE_REPEAT);
        else if (key == '2')
            GManagerComponent::HandleEventOnMenu(MENU_DRAW_SOLUTION_ANIMATION);
        else if (key == 't')
            GManagerComponent::HandleEventOnMenu(MENU_DRAW_TREE);

        else if (key == 'y')
            OnSolve();


        if (HasAllFlags((*items)[MENU_DRAW_SOLUTION_SAVED_STATES]->GetFlags(), GMenuItem::FLAG_ON)) {
            if (key == 's' && m_solutionIndex >= 0 && m_solutionIndex < seq->size()) {
                m_solutionSavedStates.push_back(m_solutionIndex);
                return true;
            }
        }

        return GManagerComponent::HandleEventOnNormalKeyPress(key);
    }

    int GManagerMP::PrepareMenu(void) {
        GManagerComponent::PrepareMenu();

        auto items = GetManager()->GetMenuItems();

        MENU_SOLVE_ONCE = items->size();
        items->push_back(new GMenuItem("solve once", GMenuItem::FLAG_OFF));
        MENU_SOLVE_REPEAT = items->size();
        items->push_back(new GMenuItem("solve repeat", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_SOLUTION_TRAJECTORY = items->size();
        items->push_back(new GMenuItem("draw solution", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_SOLUTION_ANIMATION = items->size();
        items->push_back(new GMenuItem("animate solution", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_SOLUTION_SAVED_STATES = items->size();
        items->push_back(new GMenuItem("draw saved states solution", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_SOLUTION_TIMES = items->size();
        items->push_back(new GMenuItem("show time along solution", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_TREE = items->size();
        items->push_back(new GMenuItem("draw tree", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));

        GetManager()->AddMenuEntry((*items)[MENU_SOLVE_ONCE]->GetExtendedName(), MENU_SOLVE_ONCE);
        GetManager()->AddMenuEntry((*items)[MENU_SOLVE_REPEAT]->GetExtendedName(), MENU_SOLVE_REPEAT);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_SOLUTION_TRAJECTORY]->GetExtendedName(),
                                   MENU_DRAW_SOLUTION_TRAJECTORY);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_SOLUTION_ANIMATION]->GetExtendedName(),
                                   MENU_DRAW_SOLUTION_ANIMATION);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_SOLUTION_SAVED_STATES]->GetExtendedName(),
                                   MENU_DRAW_SOLUTION_SAVED_STATES);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_SOLUTION_TIMES]->GetExtendedName(), MENU_DRAW_SOLUTION_TIMES);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_TREE]->GetExtendedName(), MENU_DRAW_TREE);

        m_menuFirstItem = MENU_SOLVE_ONCE;
        m_menuLastItem = MENU_DRAW_TREE;

        return m_menu;
    }

    void GManagerMP::OnSolve(void) {
        Timer::Clock clk;
        if (GetSetup()->GetMP() && !GetSetup()->GetMP()->IsSolved()) {
            Timer::Start(clk);
            if (m_canBeSolved) {
                GetSetup()->GetMP()->Solve(1, 7.0, m_canBeSolved);
                Stats::GetSingleton()->AddValue("TimeSolve", Timer::Elapsed(clk));
                if (GetSetup()->GetMP()->GetSolution(m_solution)) {
                    m_solutionIndex = 0;
                    for (int i = 0; i < GetSetup()->GetProblem()->m_gShowIds.size(); ++i)
                        GetSetup()->GetProblem()->m_gShowIds[i] = i;
                }

                Solution sol;
                int solved = 0;
                solved = GetSetup()->GetMP()->GetSolution(sol);
                auto a = *sol.GetStateSequence();
                auto b = *sol.GetVelocitySequence();
                auto c = *sol.GetAngleSequence();

                std::vector<std::vector<double>> sols;
                for (auto & i : a) {
                    std::vector<double> tmp;
                    tmp.push_back(i[0]);
                    tmp.push_back(i[1]);
                    sols.push_back(tmp);
//                    Logger::m_out << "x: " << i[0] << " y:" << i[1] << std::endl;
                }

                Logger::m_out << "[runtime = " << Stats::GetSingleton()->GetValue("TimeSolve") << "]"
                              << "[solved = " << m_solution.GetStateSequence()->size() << "]"
                              << "[distance = " << m_solution.GetCost() << "]"
                              << "[duration = " << m_solution.GetEndTime() << "]" << std::endl;

            }

        }
    }
}
