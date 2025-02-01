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

#ifndef Antipatrea__MP_HPP_
#define Antipatrea__MP_HPP_

#include "SceneAndSimulation/Simulator.hpp"
#include "Components/Problem.hpp"
#include "Components/Decomposition.hpp"
#include "Solution.hpp"

namespace Antipatrea {
    class MP : public Component,
               public SimulatorContainer,
               public ProblemContainer,
               public DecompositionContainer {
    public:
        MP(void) : Component(),
                   SimulatorContainer(),
                   ProblemContainer(),
                   DecompositionContainer(),
                   m_usePredictions(false) {
        }

        virtual ~MP(void) {
        }

        virtual void Info(void) const {
            Component::Info();
            Logger::m_out << " Simulator     = " << Name(GetSimulator()) << std::endl
                          << " Problem       = " << Name(GetProblem()) << std::endl
                          << " Decomposition = " << Name(GetDecomposition()) << std::endl;
        }

        virtual bool CheckSetup(void) const {
            return Component::CheckSetup() &&
                   GetSimulator() &&
                   GetProblem() &&
                   GetDecomposition() &&
                   GetSimulator()->CheckSetup() &&
                   GetProblem()->CheckSetup() &&
                   GetDecomposition()->CheckSetup();
        }

        virtual void SetupFromParams(Params &params) {
            Component::SetupFromParams(params);
            m_usePredictions = params.GetValueAsBool("UsePredictions", m_usePredictions);
        }

        virtual bool Start(void) = 0;

        virtual bool Solve(const int nrIters, const double tmax, bool &canBeSolved) = 0;

        virtual bool IsSolved(void) = 0;

        virtual bool GetSolution(Solution &sol) = 0;

        virtual void Draw(void) const = 0;

        virtual const double *GetState(const int i) const = 0;

        bool m_usePredictions;

    };

    ClassContainer(MP, m_mp);
}

#endif
