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

#ifndef Antipatrea__MPTreeVertex_HPP_
#define Antipatrea__MPTreeVertex_HPP_

#include "Components/Group.hpp"
#include "Utils/Constants.hpp"
#include "Utils/TreeVertex.hpp"

namespace Antipatrea {
    class MPTreeVertex
            : public TreeVertex<Id>, public GroupContainer {
    public:
        MPTreeVertex(void)
                : TreeVertex<Id>(), GroupContainer(), m_state(NULL), m_goal(Constants::ID_UNDEFINED),
                  m_rid(Constants::ID_UNDEFINED), m_time(0.0), m_distance(0.0), m_velocity(0.0), m_weight(0.0),
                  m_nextWaypt(Constants::ID_UNDEFINED) {
        }

        std::vector<double> to_goal_cost_;
        std::vector<double> obs_cost_;
        std::vector<double> speed_cost_;
        std::vector<double> path_cost_;

        virtual ~MPTreeVertex(void) {
            if (m_state)
                delete[] m_state;
        }

        virtual const double *GetState(void) const {
            return m_state;
        }

        virtual double *GetState(void) {
            return m_state;
        }

        virtual int GetReachedGoal(void) const {
            return m_goal;
        }

        virtual Id GetRegion(void) const {
            return m_rid;
        }

        virtual double GetTime(void) const {
            return m_time;
        }

        virtual double GetVelocity(void) const {
            return m_velocity;
        }

        virtual double GetWeight(void) const {
            return m_weight;
        }

        virtual double GetAngleVelocity(void) const {
            return m_angleVelocity;
        }

        virtual double GetDistance(void) const {
            return m_distance;
        }

        virtual int GetNextWaypt(void) const {
            return m_nextWaypt;
        }

        virtual void SetState(double s[]) {
            m_state = s;
        }

        virtual void SetReachedGoal(const int gid) {
            m_goal = gid;
        }

        virtual void SetRegion(const Id rid) {
            m_rid = rid;
        }

        virtual void SetTime(const double t) {
            m_time = t;
        }

        virtual void SetVelocity(const double s) {
            m_velocity = s;
        }

        virtual void SetAngleVelocity(const double a) {
            m_angleVelocity = a;
        }

        virtual void SetDistance(const double d) {
            m_distance = d;
        }

        virtual void SetWeight(const double d) {
            m_weight = d;
        }

        virtual void SetNextWaypt(const int waypt) {
            m_nextWaypt = waypt;
        }

        std::vector<double> v;
        std::vector<double> w;

    protected:
        double *m_state;
        int m_goal;
        Id m_rid;
        double m_time;
        double m_distance;
        double m_velocity;
        double m_angleVelocity;
        double m_weight;
        int m_nextWaypt;


    };

}

#endif
