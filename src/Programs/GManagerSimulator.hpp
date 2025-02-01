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

#ifndef Antipatrea__GManagerSimulator_HPP_
#define Antipatrea__GManagerSimulator_HPP_

#include "Utils/GManagerComponent.hpp"
#include "Programs/Setup.hpp"
#include "Programs/Constants.hpp"
#include "Utils/GTexture.hpp"

namespace Antipatrea
{

class GManagerSimulator : public GManagerComponent,
                          public SetupContainer
{
  public:
    GManagerSimulator(void) : GManagerComponent(),
                              SetupContainer()
    {
        m_menuName = "Simulator";
        m_target[0] = m_target[1] = m_target[2] = INFINITY;
	m_cfgCounter = 0;
	m_cfgs[0] = m_cfgs[1] = NULL;
    }

    virtual ~GManagerSimulator(void)
    {
	if(m_cfgs[0])
	    delete[] m_cfgs[0];
	if(m_cfgs[1])
	    delete[] m_cfgs[1];
    }

    virtual const GTexture* GetTexture(void) const
    {
        return &m_gTexture;
    }

    virtual GTexture* GetTexture(void)
    {
        return &m_gTexture;
    }

    virtual void SetupFromParams(Params &params)
    {
        GManagerComponent::SetupFromParams(params);
        m_gTexture.SetFileName(params.GetValue(Constants::KW_TextureObstaclesFile));
    }
    

    virtual void HandleEventOnDisplay(void);

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y);
 
    virtual void HandleEventOnTimer(void);

    virtual int PrepareMenu(void);

  protected:
    int MENU_STEER;
    int MENU_PATH;    
    int MENU_DRAW_OBSTACLES;
    int MENU_DRAW_STATE;
    GTexture m_gTexture;
    double m_target[3];
    double *m_cfgs[2];
    int m_cfgCounter;    
    TriMeshDefault  m_meshCfgPath;
    
};

ClassContainer(GManagerSimulator, m_gManagerSimulator);

}

#endif
