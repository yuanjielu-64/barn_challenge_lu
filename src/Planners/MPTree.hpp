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

#ifndef Antipatrea__MPTree_HPP_
#define Antipatrea__MPTree_HPP_

#include "Components/Groups.hpp"
#include "MPTreeVertex.hpp"
#include "SceneAndSimulation/Scene.hpp"
#include "Planners/Constants.hpp"
#include "Planners/MP.hpp"
#include "Utils/Constants.hpp"
#include "Utils/Tree.hpp"
#include "Utils/Grid.hpp"
#include "SceneAndSimulation/SimulatorJackal.hpp"

namespace Antipatrea {
    class MPTree : public MP {
    public:
        MPTree(void)
                : MP(), m_vidSolved(Constants::ID_UNDEFINED),
                  m_extendMinNrSteps(Constants::PLANNER_EXTEND_MIN_NR_STEPS),
                  m_extendMaxNrSteps(Constants::PLANNER_EXTEND_MAX_NR_STEPS),
                  m_extendSteerProbability(Constants::PLANNER_EXTEND_STEER_PROBABILITY),
                  m_selectNearestVertexProbability(Constants::PLANNER_SELECT_NEAREST_VERTEX_PROBABILITY),
                  m_discountSelect(Constants::PLANNER_DISCOUNT_SELECT), m_extendStopAtTarget(false),
                  m_gidCurr(Constants::ID_UNDEFINED), m_shouldWait(false),
                  STATE_X(0),
                  STATE_Y(1),
                  STATE_THETA(2),
                  STATE_VELOCITY(3),
                  STATE_ANGULAR_VELOCITY(4),
                  m_waitMaxFailures(Constants::PLANNER_WAIT_MAX_FAILURES), m_enforceOrder(false) {
        }

        virtual ~MPTree(void) {
        }

        virtual int GetExtendMinNrSteps(void) const {
            return m_extendMinNrSteps;
        }

        virtual int GetExtendMaxNrSteps(void) const {
            return m_extendMaxNrSteps;
        }

        virtual double GetExtendSteerProbability(void) const {
            return m_extendSteerProbability;
        }

        virtual double GetSelectNearestVertexProbability(void) const {
            return m_selectNearestVertexProbability;
        }

        virtual double GetDiscountSelect(void) const {
            return m_discountSelect;
        }

        virtual int GetWaitMaxFailures(void) const {
            return m_waitMaxFailures;
        }

        virtual void SetExtendMinNrSteps(const int n) {
            m_extendMinNrSteps = n;
        }

        virtual void SetExtendMaxNrSteps(const int n) {
            m_extendMaxNrSteps = n;
        }

        virtual void SetExtendSteerProbability(const double b) {
            m_extendSteerProbability = b;
        }

        virtual void SetSelectNearestVertexProbability(const double b) {
            m_selectNearestVertexProbability = b;
        }

        virtual void SetDiscountSelect(const double dsel) {
            m_discountSelect = dsel;
        }

        virtual void SetWaitMaxFailures(const int n) {
            m_waitMaxFailures = n;
        }


        virtual void SetupFromParams(Params &params) {
            MP::SetupFromParams(params);
            SetExtendMinNrSteps(params.GetValueAsInt(Constants::KW_ExtendMinNrSteps, GetExtendMinNrSteps()));
            SetExtendMaxNrSteps(params.GetValueAsInt(Constants::KW_ExtendMaxNrSteps, GetExtendMaxNrSteps()));
            SetExtendSteerProbability(
                    params.GetValueAsDouble(Constants::KW_ExtendSteerProbability, GetExtendSteerProbability()));
            SetSelectNearestVertexProbability(params.GetValueAsDouble(Constants::KW_SelectNearestVertexProbability,
                                                                      GetSelectNearestVertexProbability()));
            SetDiscountSelect(params.GetValueAsDouble(Constants::KW_DiscountSelect, GetDiscountSelect()));
            SetWaitMaxFailures(params.GetValueAsInt(Constants::KW_WaitMaxFailures, GetWaitMaxFailures()));
        }

        virtual bool Start(void);

        virtual bool IsSolved(void) {
            return m_vidSolved >= 0;
        }

        virtual bool GetSolution(Solution &sol);

        virtual void Draw(void) const;


        virtual const double *GetState(const int vid) const {
            return dynamic_cast<const MPTreeVertex *>(m_tree.GetVertex(vid))->GetState();
        }


    protected:
        enum ExtendStatus {
            EXTEND_NORMAL = 0,
            EXTEND_COLLISION = 1,
            EXTEND_TARGET = 2,
            EXTEND_GOAL = 3
        };

        virtual MPTreeVertex *NewVertex(void) const {
            return new MPTreeVertex();
        }

        virtual ExtendStatus ExtendFrom(const Id vid, const double p[]);

        virtual void ExtendWait(void);

        virtual bool SetupVertex(MPTreeVertex &v);

        virtual void AddVertex(MPTreeVertex *const v);

        virtual bool UpdateGroups(MPTreeVertex &v);

        virtual Group *NewGroup(void) const {
            return new Group();
        }

        virtual GroupKey *NewGroupKey(void) const = 0;

        virtual bool CompleteGroup(Group &group, MPTreeVertex &v) {
            return true;
        }

        virtual double SetBoundary(double x) {
            double min = GetSimulator()->GetScene()->GetGrid()->GetMin()[0];
            double max = GetSimulator()->GetScene()->GetGrid()->GetMax()[0];

            x = (std::min(min, x) == x) ? min + 1 : x;
            x = (std::max(max, x) == x) ? max - 1 : x;

            return x;
        }

        virtual void InitializeGroups(void);

        virtual Id SelectVertex(const Group &g, const double target[] = NULL) const {
            return SelectVertex(g.GetVertices()->size(), &(g.GetVertices()->operator[](0)), target);
        }

        virtual Id SelectVertex(const int n, const Id vids[], const double target[] = NULL) const;

        Tree<Id> m_tree;
        Id m_vidSolved;
        int m_extendMaxNrSteps;
        int m_extendMinNrSteps;
        double m_extendSteerProbability;
        double m_selectNearestVertexProbability;
        double m_discountSelect;
        bool m_extendStopAtTarget;
        Groups m_groups;
        int m_gidCurr;
        bool m_shouldWait;
        int m_waitMaxFailures;
        int m_nrGroupsWhileWaiting;
        bool m_enforceOrder;

        Robot_config *robot;

        int STATE_X;
        int STATE_Y;
        int STATE_THETA;
        int STATE_VELOCITY;
        int STATE_ANGULAR_VELOCITY;
    };

    ClassContainer(MPTree, m_mpTree);
}

#endif
