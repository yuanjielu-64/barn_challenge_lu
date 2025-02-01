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

#ifndef Antipatrea__GManagerMP_HPP_
#define Antipatrea__GManagerMP_HPP_

#include "Utils/GManagerComponent.hpp"
#include "Programs/Setup.hpp"
#include "Planners/Solution.hpp"
#include "Utils/Constants.hpp"

namespace Antipatrea {

    class GManagerMP : public GManagerComponent,
                       public SetupContainer {
    public:
        GManagerMP(void) : GManagerComponent(),
                           SetupContainer(),
                           m_solutionIndex(Constants::ID_UNDEFINED) {
            m_menuName = "Motion Planner";
            m_canBeSolved = true;

        }

        virtual ~GManagerMP(void) {
        }

        virtual void HandleEventOnDisplay(void);

        virtual void HandleEventOnTimer(void);

        virtual bool HandleEventOnMenu(const int item);

        virtual bool HandleEventOnNormalKeyPress(const int key);

        virtual int PrepareMenu(void);

    protected:
        virtual void OnSolve(void);

        bool m_canBeSolved;

        int MENU_SOLVE_ONCE;
        int MENU_SOLVE_REPEAT;
        int MENU_DRAW_SOLUTION_TRAJECTORY;
        int MENU_DRAW_SOLUTION_ANIMATION;
        int MENU_DRAW_SOLUTION_SAVED_STATES;
        int MENU_DRAW_SOLUTION_TIMES;
        int MENU_DRAW_TREE;

        Solution m_solution;
        int m_solutionIndex;
        std::vector<int> m_solutionSavedStates;
    };

    ClassContainer(GManagerMP, m_gManagerMP);

}

#endif
