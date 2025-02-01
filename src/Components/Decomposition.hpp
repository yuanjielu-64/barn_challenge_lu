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

#ifndef Antipatrea__Decomposition_HPP_
#define Antipatrea__Decomposition_HPP_

#include "Components/Region.hpp"
#include "Components/DecompositionEdge.hpp"
#include "SceneAndSimulation/Simulator.hpp"
#include "Components/Problem.hpp"
#include "Utils/Graph.hpp"

namespace Antipatrea {
    class Decomposition : public Component,
                          public SimulatorContainer,
                          public ProblemContainer {
    public:
        Decomposition(void) : Component(),
                              SimulatorContainer(),
                              ProblemContainer(),
                              m_minRegionClearance(INFINITY),
                              m_maxRegionClearance(0.0),
                              m_predictionsComputed(false) {
        }

        virtual ~Decomposition(void) {
        }

        virtual void Info(void) const {
            Component::Info();
            Logger::m_out << " Simulator = " << Name(GetSimulator()) << std::endl
                          << " Problem   = " << Name(GetProblem()) << std::endl;
        }

        virtual bool CheckSetup(void) const {
            return Component::CheckSetup() &&
                   GetSimulator() &&
                   GetProblem() &&
                   GetSimulator()->CheckSetup() &&
                   GetProblem()->CheckSetup();
        }

        virtual const Graph<Id> *GetGraph(void) const {
            return &m_graph;
        }

        virtual Graph<Id> *GetGraph(void) {
            return &m_graph;
        }

        virtual void Construct(const double tmax) = 0;

        virtual bool Construct() = 0;

        virtual Id LocateRegion(const double cfg[]) = 0;

        virtual void PathsToGoals(void);

        virtual void PathsToGoalsRadix(void);

        virtual void DrawRegionsCfg(const Flags flags = 0);

        virtual void DrawEdges(const Flags flags = 0);

        virtual void DrawEdge(const Id keyFrom, const Id keyTo);

        virtual void SetRegionColors(const double rgb[], const Flags flags = 0) {
            SetRegionColors(rgb[0], rgb[1], rgb[2], flags);
        }

        virtual void SetRegionColors(const double r, const double g, const double b, const Flags flags = 0);

        virtual void SetRegionColorsBasedOnClearance(const Flags flags = 0);

        virtual void SetRegionColorsBasedOnPathCost(const int goalIndex, const Flags flags = 0);

        virtual void PathFromIdsToPoints(const int n, const Id rids[], std::vector<double> &pts);

        virtual bool AreInitialAndGoalsConnected(void);

        virtual void AddGoals(void);

    protected:
        virtual void SetInitialAndGoalsPairs(const int keyInit);



        virtual void CreateRepresentativeCfg(RegionGeometric &r, const bool clearance = true);

        virtual void UpdateMinMaxRegionClearances(const double c);

        Graph<Id> m_graph;
        double m_minRegionClearance;
        double m_maxRegionClearance;
        std::vector<double> m_maxPathCosts;
        std::vector<int> m_pairsInitialAndGoalsUnconnected;

        bool m_predictionsComputed;

    };

    ClassContainer(Decomposition, m_decomposition);
}

#endif
