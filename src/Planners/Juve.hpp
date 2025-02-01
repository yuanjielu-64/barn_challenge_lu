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

#ifndef Antipatrea__Juve_HPP_
#define Antipatrea__Juve_HPP_

#include "Components/Follow.hpp"
#include "Components/GroupKeyTour.hpp"
#include "Components/GroupSelector.hpp"
#include "Components/TourGenerator.hpp"
#include "Planners/MPTree.hpp"

namespace Antipatrea {
    class Juve
            : public MPTree, public GroupSelectorContainer, public TourGeneratorContainer {
    public:
        Juve(void)
                : MPTree(), GroupSelectorContainer(), TourGeneratorContainer(),
                  m_counterFromExploreToFollow(Constants::COUNTER_FROM_EXPLORE_TO_FOLLOW),
                  m_counterFromFollowToExplore(Constants::COUNTER_FROM_FOLLOW_TO_EXPLORE) {
        }

        virtual ~Juve(void) {
        }

        virtual int GetCounterFromExploreToFollow(void) const {
            return m_counterFromExploreToFollow;
        }

        virtual int GetCounterFromFollowToExplore(void) const {
            return m_counterFromFollowToExplore;
        }

        virtual void SetCounterFromExploreToFollow(const int counter) {
            m_counterFromExploreToFollow = counter;
        }

        virtual void SetCounterFromFollowToExplore(const int counter) {
            m_counterFromFollowToExplore = counter;
        }

        virtual void SetupFromParams(Params &params) {
            MPTree::SetupFromParams(params);
            SetCounterFromExploreToFollow(
                    params.GetValueAsInt(Constants::KW_CounterFromExploreToFollow, GetCounterFromExploreToFollow()));
            SetCounterFromFollowToExplore(
                    params.GetValueAsInt(Constants::KW_CounterFromFollowToExplore, GetCounterFromFollowToExplore()));

            auto data = params.GetData(Constants::KW_Follow);
            if (data && data->m_params)
                m_follow.SetupFromParams(*(data->m_params));
        }

        virtual bool Solve(const int nrIters, const double tmax, bool &canBeSolved);

        virtual void Draw(void) const;

    protected:
        virtual void AddVertex(MPTreeVertex *const v);

        virtual bool SetupVertex(MPTreeVertex &v);

        virtual GroupKey *NewGroupKey(void) const {
            return new GroupKeyTour();
        }

        virtual bool CompleteGroup(Group &group, MPTreeVertex &v);

        virtual void AllPtsInTour(GroupKeyTour &tkey);


        struct Progress {
            Progress(void)
                    : m_nrIters(0), m_nrGroups(0), m_nrItersStart(300), m_nrItersEnd(600) {
            }

            int m_nrIters;
            int m_nrGroups;
            int m_nrItersStart;
            int m_nrItersEnd;
        };

        Follow m_follow;
        Groups m_followGroups;
        bool m_followSolved;
        int m_counterFromFollowToExplore;
        int m_counterFromExploreToFollow;
        Progress m_followProgress;

        virtual void FollowClear(void);

        virtual void FollowUpdateIfNoProgress(void);

        virtual void FollowGenerateWaypts(void);

        virtual void FollowUpdateGroups(MPTreeVertex &v);

        Group *m_groupFollowOriginator;

        std::vector<double> m_gTourPts;
        std::vector<int> m_gTourGoals;

    };

    ClassContainer(Juve, m_juve);
}

#endif
