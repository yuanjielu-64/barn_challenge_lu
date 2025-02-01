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

#ifndef Antipatrea__ProgramsSetup_HPP_
#define Antipatrea__ProgramsSetup_HPP_

#include "Components/Decomposition.hpp"
#include "Components/GroupSelector.hpp"
#include "Components/Problem.hpp"
#include "SceneAndSimulation/Scene.hpp"
#include "SceneAndSimulation/Simulator.hpp"
#include "Components/TourGenerator.hpp"
#include "Robot/Jackal.hpp"
#include "Planners/MP.hpp"

namespace Antipatrea {
    class Setup
            : public SceneContainer,
              public SimulatorContainer,
              public DecompositionContainer,
              public ProblemContainer,
              public GroupSelectorContainer,
              public TourGeneratorContainer,
              public MPContainer {
    public:
        Setup(void);

        virtual ~Setup(void) {
            if (GetMP())
                delete GetMP();
            if (GetGroupSelector())
                delete GetGroupSelector();
            if (GetTourGenerator())
                delete GetTourGenerator();
            if (GetDecomposition())
                delete GetDecomposition();
            if (GetProblem())
                delete GetProblem();
            if (GetScene())
                delete GetScene();
            if (GetSimulator())
                delete GetSimulator();

        }

        enum {
            COLOR_OBSTACLES = 0,
            COLOR_DECOMPOSITION_REGIONS_CFG,
            COLOR_DECOMPOSITION_REGIONS_BORDER,
            COLOR_DECOMPOSITION_REGIONS_INSIDE,
            COLOR_DECOMPOSITION_SELECT_REGION,
            COLOR_DECOMPOSITION_REGIONS_EDGES,
            COLOR_GOALS_REGIONS_CFG,
            COLOR_GOALS_REGIONS_BORDER,
            COLOR_GOALS_REGIONS_INSIDE,
            COLOR_GOALS_REGIONS_EDGES,
            COLOR_PATH_TO_GOAL,
            COLOR_STEER,
            COLOR_MP,
            COLOR_SOLUTION,
            NR_COLORS,
            NR_COLOR_COMPONENTS = 3
        };

        virtual int GetNrColors(void) const {
            return NR_COLORS;
        }

        virtual const double *GetColor(const int i) const {
            return &m_colors[NR_COLOR_COMPONENTS * i];
        }

        virtual double *GetColor(const int i) {
            return &m_colors[NR_COLOR_COMPONENTS * i];
        }

        virtual void UpdateFromParams(Params &p, Robot_config &robot, int &robot_state);

        virtual void SetupFromParams(Params &p, Robot_config &robot);

        virtual void RunConstructDecomposition(Params &p);

        virtual void RunInitialState(Params &p);

        virtual void RunAdjustTimeBounds(Params &p);

    protected:
        double m_colors[NR_COLOR_COMPONENTS * NR_COLORS];
    };

    ClassContainer(Setup, m_setup);
}

#endif
