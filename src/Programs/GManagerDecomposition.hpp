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

#ifndef Antipatrea__GManagerDecomposition_HPP_
#define Antipatrea__GManagerDecomposition_HPP_

#include "Utils/GManagerComponent.hpp"
#include "Programs/Setup.hpp"
#include "Programs/Constants.hpp"
#include "Utils/GTexture.hpp"

namespace Antipatrea
{

class GManagerDecomposition : public GManagerComponent,
                              public SetupContainer
{
  public:
    GManagerDecomposition(void) : GManagerComponent(),
                                  SetupContainer()
    {
        m_menuName = "Decomposition";
        m_target[0] = m_target[1] = m_target[2] = INFINITY;
        m_idSelectedRegion = Constants::ID_UNDEFINED;
        m_indexSelectedGoal = 0;
    }

    virtual ~GManagerDecomposition(void)
    {
    }

    virtual void HandleEventOnDisplay(void);

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y);

    virtual bool HandleEventOnMenu(const int item);

    virtual bool HandleEventOnNormalKeyPress(const int key);

    virtual int PrepareMenu(void);

    virtual int GetSelectedGoal(void) const
    {
        return m_indexSelectedGoal;
    }

    virtual Id GetSelectedRegion(void) const
    {
        return m_idSelectedRegion;
    }

  protected:
    int MENU_DRAW_REGIONS_CFG;
    int MENU_DRAW_REGIONS_BORDER;
    int MENU_DRAW_REGIONS_INSIDE;
    int MENU_DRAW_REGIONS_EDGES;
    int MENU_DRAW_REGIONS_COLOR_SAME;
    int MENU_DRAW_REGIONS_COLOR_CLEARANCE;
    int MENU_DRAW_REGIONS_COLOR_PATH;
    int MENU_SELECT_REGION;

    int m_idSelectedRegion;
    int m_indexSelectedGoal;
    double m_target[3];
};

ClassContainer(GManagerDecomposition, m_gManagerDecomposition);

}

#endif
