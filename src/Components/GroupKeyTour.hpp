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

#ifndef Antipatrea__GroupKeyTour_HPP_
#define Antipatrea__GroupKeyTour_HPP_

#include "Components/GroupKeyRegion.hpp"
#include "Components/GroupKeyUnreachedGoals.hpp"
#include "Components/Tour.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Algebra.hpp"

namespace Antipatrea {

    class GroupKeyTour : public GroupKey {
    public:
        GroupKeyTour(void) : GroupKey() {
        }

        virtual ~GroupKeyTour(void) {
        }

        virtual const GroupKeyRegion *GetRegionKey(void) const {
            return &m_keyRegion;
        }

        virtual GroupKeyRegion *GetRegionKey(void) {
            return &m_keyRegion;
        }

        virtual const GroupKeyUnreachedGoals *GetUnreachedGoalsKey(void) const {
            return &m_keyUnreachedGoals;
        }

        virtual GroupKeyUnreachedGoals *GetUnreachedGoalsKey(void) {
            return &m_keyUnreachedGoals;
        }

        virtual const Tour *GetTour(void) const {
            return &m_tour;
        }

        virtual Tour *GetTour(void) {
            return &m_tour;
        }

        virtual void SetContent(const GroupKey &key) {
            auto tkey = dynamic_cast<const GroupKeyTour &>(key);
            m_keyRegion.SetContent(*(tkey.GetRegionKey()));
            m_keyUnreachedGoals.SetContent(*(tkey.GetUnreachedGoalsKey()));
        }

        virtual void SetContent(const Problem &prob) {
            m_keyRegion.SetContent(prob);
            m_keyUnreachedGoals.SetContent(prob);
        }

        virtual void UpdateContent(const MPTreeVertex &v) {

            m_keyRegion.UpdateContent(v);
            m_keyUnreachedGoals.UpdateContent(v);
            SetState(const_cast<double *>(v.GetState()));
            if (m_tour.GetStartTime() < 0)
                m_tour.SetStartTime(v.GetTime());
        }

        virtual bool SameContent(const GroupKey &key) const {
            auto tkey = dynamic_cast<const GroupKeyTour &>(key);
            bool flag;

            if (Algebra::PointDistance(2, GetState(), tkey.GetState()) < 0.5 &&
                fabs(GetState()[2] - tkey.GetState()[2]) < 0.4 &&
                fabs(GetState()[3] - tkey.GetState()[3]) < 0.4)
                flag = true;
            else
                flag = false;

            return flag && m_keyUnreachedGoals.SameContent(*(tkey.GetUnreachedGoalsKey())) &&
                   m_keyRegion.SameContent(*(tkey.GetRegionKey())) &&
                   m_tour.IsFeasible(tkey.GetTour()->GetStartTime());
        }

        virtual void SetState(double s[]) {
            m_state = s;
        }

        virtual const double *GetState(void) const {
            return m_state;
        }

        virtual double *GetState(void) {
            return m_state;
        }

        virtual bool IsSolved(void) const {
            return m_keyRegion.IsSolved() ||
                   m_keyUnreachedGoals.IsSolved();
        }

        virtual void Print(std::ostream &out) const {
            out << "<" << m_keyRegion << "> <" << m_keyUnreachedGoals << "> <" << m_tour << ">";
        }

    protected:
        GroupKeyRegion m_keyRegion;
        GroupKeyUnreachedGoals m_keyUnreachedGoals;
        Tour m_tour;
        double *m_state;
    };

    ClassContainer(GroupKeyTour, m_groupKeyTour);
}

#endif
