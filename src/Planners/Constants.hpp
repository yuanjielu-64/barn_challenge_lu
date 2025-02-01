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

#ifndef Antipatrea__PlannersConstants_HPP_
#define Antipatrea__PlannersConstants_HPP_

#include "Utils/Constants.hpp"

namespace Antipatrea {
    namespace Constants {

        const int PLANNER_EXTEND_MIN_NR_STEPS = 20;
        const int PLANNER_EXTEND_MAX_NR_STEPS = 40;
        const double PLANNER_EXTEND_STEER_PROBABILITY = 0.9;
        const double PLANNER_SELECT_NEAREST_VERTEX_PROBABILITY = 0.9;
        const double PLANNER_FOLLOW_RADIUS = 3.0;
        const double PLANNER_DISCOUNT_SELECT = 0.95;
        const int PLANNER_WAIT_MAX_FAILURES = 3;
        const double DISTANCE_ONE_STEP_GOAL = 1;

        const double GROUP_MAX_WEIGHT = 10000000000000000000000.0;

        const int COUNTER_FROM_EXPLORE_TO_FOLLOW = 20;
        const int COUNTER_FROM_FOLLOW_TO_EXPLORE = 2;

        const char KW_ExtendMinNrSteps[] = "ExtendMinNrSteps";
        const char KW_ExtendMaxNrSteps[] = "ExtendMaxNrSteps";
        const char KW_ExtendSteerProbability[] = "ExtendSteerProbability";
        const char KW_SelectNearestVertexProbability[] = "SelectNearestVertexProbability";
        const char KW_DiscountSelect[] = "DiscountSelect";
        const char KW_FollowRadius[] = "FollowRadius";
        const char KW_CounterFromExploreToFollow[] = "CounterFromExploreToFollow";
        const char KW_CounterFromFollowToExplore[] = "CounterFromFollowToExplore";
        const char KW_WaitMaxFailures[] = "WaitMaxFailures";

        const char KW_Milan[] = "Milan";
        const char KW_Juve[] = "Juve";
        const char KW_Dromos[] = "Dromos";
        const char KW_DDP[] = "DDP";
        const char KW_DWAPlanner[] = "DWAPlanner";
        const char KW_DDPDWAPlanner[] = "DDPDWAPlanner";
        const char KW_MPPIPlanner[] = "MPPIPlanner";
        const char KW_DDPMPPIPlanner[] = "DDPMPPIPlanner";
        const char KW_Sequential[] = "Sequential";
        const char KW_RRT[] = "RRT";






    }


}

#endif
