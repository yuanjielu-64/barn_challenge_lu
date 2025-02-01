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
#include "Programs/GManagerSimulator.hpp"
#include "Utils/GManager.hpp"
#include "Utils/Misc.hpp"

namespace Antipatrea {
    void GManagerSimulator::HandleEventOnDisplay(void) {
        GManagerComponent::HandleEventOnDisplay();

        m_gTexture.AutomaticCoords();
        m_gTexture.Use();

        auto scene = GetSetup()->GetScene();
        auto sim = GetSetup()->GetSimulator();
        auto items = GetManager()->GetMenuItems();

        if (scene) {
            const double *pmin = scene->GetGrid()->GetMin();
            const double *pmax = scene->GetGrid()->GetMax();
            const double off = 0.002;

            GetManager()->SetValue(GManager::INDEX_MINX, pmin[0] - off * (pmax[0] - pmin[0]));
            GetManager()->SetValue(GManager::INDEX_MINY, pmin[1] - off * (pmax[1] - pmin[1]));
            GetManager()->SetValue(GManager::INDEX_MAXX, pmax[0] + off * (pmax[0] - pmin[0]));
            GetManager()->SetValue(GManager::INDEX_MAXY, pmax[1] + off * (pmax[1] - pmin[1]));

            if (HasAllFlags((*items)[MENU_DRAW_OBSTACLES]->GetFlags(), GMenuItem::FLAG_ON)) {
                GDrawColor(GetSetup()->GetColor(Setup::COLOR_OBSTACLES));
                GDrawWireframe(false);
                scene->DrawObstacles();

                m_setup->GetProblem()->DrawGoals();

            }
        }

        GMaterial gmat;
        gmat.SetPearl();
        GDrawMaterial(gmat);

        //GDrawColor(1, 1, 1);
        scene->DrawTerrain();

        if (sim && HasAllFlags((*items)[MENU_DRAW_STATE]->GetFlags(), GMenuItem::FLAG_ON))
            sim->DrawState();

        if (m_target[0] != INFINITY && HasAllFlags((*items)[MENU_STEER]->GetFlags(), GMenuItem::FLAG_ON)) {
            GDrawColor(GetSetup()->GetColor(Setup::COLOR_STEER));
            GDrawSphere3D(m_target, sim ? sim->GetDistanceReachedSteerPosition()
                                        : Constants::SIMULATOR_DISTANCE_REACHED_STEER_POSITION);
        }


        if (m_cfgs[0] != NULL && m_cfgs[1] != NULL &&
            HasAllFlags((*items)[MENU_PATH]->GetFlags(), GMenuItem::FLAG_ON)) {
            GetSetup()->GetSimulator()->DrawCfg(m_cfgs[0]);
            GetSetup()->GetSimulator()->DrawCfg(m_cfgs[1]);
            m_meshCfgPath.Draw();
        }
    }

    bool GManagerSimulator::HandleEventOnMouseLeftBtnDown(const int x, const int y) {
        auto items = GetManager()->GetMenuItems();

        if (GetSetup()->GetSimulator() && HasAllFlags((*items)[MENU_STEER]->GetFlags(), GMenuItem::FLAG_ON)) {
            GetSetup()->GetSimulator()->SamplePosition(m_target);
            GetManager()->MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
            GetSetup()->GetSimulator()->StartSteerToPosition(m_target);

            Logger::m_out << "steering target: " << m_target[0] << " " << m_target[1] << " " << m_target[2]
                          << std::endl;

            return true;
        }

        if (GetSetup()->GetSimulator() && HasAllFlags((*items)[MENU_PATH]->GetFlags(), GMenuItem::FLAG_ON)) {
            if (m_cfgs[0] == NULL)
                m_cfgs[0] = GetSetup()->GetSimulator()->GetCfgAllocator()->New();
            if (m_cfgs[1] == NULL)
                m_cfgs[1] = GetSetup()->GetSimulator()->GetCfgAllocator()->New();

            double p[3];
            GetSetup()->GetSimulator()->SampleCfg(m_cfgs[m_cfgCounter]);
            GetManager()->MousePosFromScreenToWorld(x, y, &p[0], &p[1], &p[2]);
            GetSetup()->GetSimulator()->SetPositionCfg(p, m_cfgs[m_cfgCounter]);
            m_cfgCounter = (m_cfgCounter + 1) % 2;


            const double oneStepDist = 1.0;
            auto dt = AdjustTimeStep(oneStepDist / GetSetup()->GetSimulator()->DistanceCfgs(m_cfgs[0], m_cfgs[1]));
            m_meshCfgPath.Clear();
            GetSetup()->GetSimulator()->AddToMeshPathCfgs(m_meshCfgPath, m_cfgs[0], m_cfgs[1], dt, dt, 1.0 - dt);

            return true;
        }

        return GManagerComponent::HandleEventOnMouseLeftBtnDown(x, y);
    }

    void GManagerSimulator::HandleEventOnTimer(void) {
        auto items = GetManager()->GetMenuItems();

        if (GetSetup()->GetSimulator() && HasAllFlags((*items)[MENU_STEER]->GetFlags(), GMenuItem::FLAG_ON) &&
            m_target[0] != INFINITY) {
            GetSetup()->GetSimulator()->SteerToPosition(m_target, true);
            GetSetup()->GetSimulator()->SimulateOneStep();
        }

        GManagerComponent::HandleEventOnTimer();
    }

    int GManagerSimulator::PrepareMenu(void) {
        GManagerComponent::PrepareMenu();

        auto items = GetManager()->GetMenuItems();

        MENU_STEER = items->size();
        items->push_back(new GMenuItem("steer", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_PATH = items->size();
        items->push_back(new GMenuItem("path", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_OBSTACLES = items->size();
        items->push_back(new GMenuItem("draw obstacles", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_STATE = items->size();
        items->push_back(new GMenuItem("draw state", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));

        GetManager()->AddMenuEntry((*items)[MENU_STEER]->GetExtendedName(), MENU_STEER);
        GetManager()->AddMenuEntry((*items)[MENU_PATH]->GetExtendedName(), MENU_PATH);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_OBSTACLES]->GetExtendedName(), MENU_DRAW_OBSTACLES);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_STATE]->GetExtendedName(), MENU_DRAW_STATE);

        m_menuFirstItem = MENU_STEER;
        m_menuLastItem = MENU_DRAW_STATE;

        return m_menu;
    }
}
