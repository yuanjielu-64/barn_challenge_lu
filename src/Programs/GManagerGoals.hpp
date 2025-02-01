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

#ifndef Antipatrea__GManagerGoals_HPP_
#define Antipatrea__GManagerGoals_HPP_

#include "Utils/GManagerComponent.hpp"
#include "Programs/Setup.hpp"
#include "Programs/Constants.hpp"
#include "Utils/GTexture.hpp"

namespace Antipatrea {

    class GManagerGoals : public GManagerComponent,
                          public SetupContainer {
    public:
        GManagerGoals(void) : GManagerComponent(),
                              SetupContainer() {
            m_menuName = "Goals";
        }

        virtual ~GManagerGoals(void) {
        }

        virtual void HandleEventOnDisplay(void);

        virtual bool HandleEventOnMenu(const int item);

        virtual int PrepareMenu(void);

    protected:
        int MENU_DRAW_GOALS_CFG;
        int MENU_DRAW_GOALS_BORDER;
        int MENU_DRAW_GOALS_INSIDE;
        int MENU_DRAW_GOALS_EDGES;
        int MENU_DRAW_GOALS_COLOR_SAME;
        int MENU_DRAW_GOALS_COLOR_CLEARANCE;
        int MENU_DRAW_GOALS_COLOR_PATH;

    };

    ClassContainer(GManagerGoals, m_gManagerGoals);

}

#endif
