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
#include "Programs/GManagerDecomposition.hpp"
#include "Utils/GManager.hpp"
#include "Components/RegionGeometric.hpp"
#include "Components/DecompositionGeometric.hpp"

namespace Antipatrea {
    void GManagerDecomposition::HandleEventOnDisplay(void) {
        GManagerComponent::HandleEventOnDisplay();

        auto items = GetManager()->GetMenuItems();
        auto decomp = GetSetup()->GetDecomposition();
        auto sim = GetSetup()->GetSimulator();

        if (decomp == NULL)
            return;

        if (m_idSelectedRegion >= 0 && HasAllFlags((*items)[MENU_SELECT_REGION]->GetFlags(), GMenuItem::FLAG_ON)) {
            auto r = dynamic_cast<Region *>(decomp->GetGraph()->GetVertex(m_idSelectedRegion));
            GDrawColor(GetSetup()->GetColor(Setup::COLOR_DECOMPOSITION_SELECT_REGION));
            if (dynamic_cast<RegionGeometric *>(r))
                dynamic_cast<RegionGeometric *>(r)->DrawShape();
            if (sim)
                sim->DrawCfg(r->GetCfg());
            for (auto &idNeigh: *(r->GetConnections()))
                decomp->DrawEdge(m_idSelectedRegion, idNeigh);

            if (m_indexSelectedGoal >= 0 && m_indexSelectedGoal < r->GetPathDataToGoals()->size()) {
                auto path = &((*(r->GetPathDataToGoals()))[m_indexSelectedGoal]->m_path);
                for (int j = path->size() - 1; j > 0; --j)
                    decomp->DrawEdge((*path)[j - 1], (*path)[j]);
                //  Logger::m_out << " path from " << r->GetKey() << " to goal " << m_indexSelectedGoal << " has cost " <<
                //	(*(r->GetPathDataToGoals()))[m_indexSelectedGoal]->m_cost << std::endl;

                for (int j = path->size() - 1; j > 0; --j)
                    sim->DrawCfg(dynamic_cast<Region *>(decomp->GetGraph()->GetVertex((*path)[j]))->GetCfg());

            }
        }

        if (HasAllFlags((*items)[MENU_DRAW_REGIONS_CFG]->GetFlags(), GMenuItem::FLAG_ON)) {
            GDrawColor(GetSetup()->GetColor(Setup::COLOR_DECOMPOSITION_REGIONS_CFG));
            decomp->DrawRegionsCfg(Region::STATUS_FREE | Region::STATUS_REGULAR);
        }

        if (HasAllFlags((*items)[MENU_DRAW_REGIONS_INSIDE]->GetFlags(), GMenuItem::FLAG_ON) &&
            dynamic_cast<DecompositionGeometric *>(decomp)) {
            GDrawColor(GetSetup()->GetColor(Setup::COLOR_DECOMPOSITION_REGIONS_INSIDE));
            dynamic_cast<DecompositionGeometric *>(decomp)->DrawRegionsShape(
                    Region::STATUS_FREE | Region::STATUS_REGULAR);
        }

        if (HasAllFlags((*items)[MENU_DRAW_REGIONS_BORDER]->GetFlags(), GMenuItem::FLAG_ON) &&
            dynamic_cast<DecompositionGeometric *>(decomp)) {
            GDrawWireframe(true);
            GDrawColor(GetSetup()->GetColor(Setup::COLOR_DECOMPOSITION_REGIONS_BORDER));
            dynamic_cast<DecompositionGeometric *>(decomp)->DrawRegionsShape(
                    Region::STATUS_FREE | Region::STATUS_REGULAR);
            GDrawWireframe(false);
        }

        if (HasAllFlags((*items)[MENU_DRAW_REGIONS_EDGES]->GetFlags(), GMenuItem::FLAG_ON)) {
            GDrawColor(GetSetup()->GetColor(Setup::COLOR_DECOMPOSITION_REGIONS_EDGES));
            decomp->DrawEdges(DecompositionEdge::STATUS_REGULAR);
        }
    }

    bool GManagerDecomposition::HandleEventOnMouseLeftBtnDown(const int x, const int y) {
        auto items = GetManager()->GetMenuItems();

        if (GetSetup()->GetDecomposition() &&
            HasAllFlags((*items)[MENU_SELECT_REGION]->GetFlags(), GMenuItem::FLAG_ON)) {
            Logger::m_out << "................YES " << std::endl;

            GetManager()->MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
            m_idSelectedRegion = GetSetup()->GetDecomposition()->LocateRegion(m_target);
            if (m_idSelectedRegion >= 0)
                Logger::m_out << "selected region " << m_idSelectedRegion << " with status "
                              << dynamic_cast<Region *>(GetSetup()->GetDecomposition()->GetGraph()->GetVertex(
                                      m_idSelectedRegion))->GetFlags() << std::endl;
            else
                Logger::m_out << "could not select region" << std::endl;


            return true;
        }

        return GManagerComponent::HandleEventOnMouseLeftBtnDown(x, y);
    }

    bool GManagerDecomposition::HandleEventOnMenu(const int item) {
        if (item == MENU_DRAW_REGIONS_COLOR_CLEARANCE) {
            GetSetup()->GetDecomposition()->SetRegionColorsBasedOnClearance(
                    Region::STATUS_FREE | Region::STATUS_REGULAR);
            return true;
        } else if (item == MENU_DRAW_REGIONS_COLOR_PATH) {
            if (GetSetup()->GetProblem()->GetGoals()->size() > 0)
                GetSetup()->GetDecomposition()->SetRegionColorsBasedOnPathCost(
                        m_indexSelectedGoal >= 0 ? m_indexSelectedGoal : 0,
                        Region::STATUS_FREE | Region::STATUS_REGULAR);
            return true;
        } else if (item == MENU_DRAW_REGIONS_COLOR_SAME) {
            GetSetup()->GetDecomposition()->SetRegionColors(
                    GetSetup()->GetColor(Setup::COLOR_DECOMPOSITION_REGIONS_INSIDE),
                    Region::STATUS_FREE | Region::STATUS_REGULAR);
            return true;
        } else
            return GManagerComponent::HandleEventOnMenu(item);
    }

    bool GManagerDecomposition::HandleEventOnNormalKeyPress(const int key) {
        auto items = GetManager()->GetMenuItems();

        if (key == '+' && m_indexSelectedGoal < GetSetup()->GetProblem()->GetGoals()->size() - 1) {
            ++m_indexSelectedGoal;
            Logger::m_out << "..selected goal = " << m_indexSelectedGoal << std::endl;
            return true;
        } else if (key == '-' && m_indexSelectedGoal > 0) {
            --m_indexSelectedGoal;
            Logger::m_out << "..selected goal = " << m_indexSelectedGoal << std::endl;
            return true;
        } else if (key == '*')
            m_idSelectedRegion = 4113; //3092;


        return GManagerComponent::HandleEventOnNormalKeyPress(key);
    }

    int GManagerDecomposition::PrepareMenu(void) {
        GManagerComponent::PrepareMenu();

        auto items = GetManager()->GetMenuItems();

        MENU_DRAW_REGIONS_CFG = items->size();
        items->push_back(new GMenuItem("draw regions cfg", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_REGIONS_BORDER = items->size();
        items->push_back(new GMenuItem("draw regions border", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_REGIONS_INSIDE = items->size();
        items->push_back(new GMenuItem("draw regions inside", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_REGIONS_EDGES = items->size();
        items->push_back(new GMenuItem("draw regions edges", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_REGIONS_COLOR_SAME = items->size();
        items->push_back(new GMenuItem("regions color same", GMenuItem::FLAG_OFF));
        MENU_DRAW_REGIONS_COLOR_CLEARANCE = items->size();
        items->push_back(new GMenuItem("regions color clearance", GMenuItem::FLAG_OFF));
        MENU_DRAW_REGIONS_COLOR_PATH = items->size();
        items->push_back(new GMenuItem("regions color path", GMenuItem::FLAG_OFF));
        MENU_SELECT_REGION = items->size();
        items->push_back(new GMenuItem("select region", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));

        GetManager()->AddMenuEntry((*items)[MENU_DRAW_REGIONS_CFG]->GetExtendedName(), MENU_DRAW_REGIONS_CFG);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_REGIONS_BORDER]->GetExtendedName(), MENU_DRAW_REGIONS_BORDER);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_REGIONS_INSIDE]->GetExtendedName(), MENU_DRAW_REGIONS_INSIDE);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_REGIONS_EDGES]->GetExtendedName(), MENU_DRAW_REGIONS_EDGES);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_REGIONS_COLOR_SAME]->GetExtendedName(),
                                   MENU_DRAW_REGIONS_COLOR_SAME);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_REGIONS_COLOR_CLEARANCE]->GetExtendedName(),
                                   MENU_DRAW_REGIONS_COLOR_CLEARANCE);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_REGIONS_COLOR_PATH]->GetExtendedName(),
                                   MENU_DRAW_REGIONS_COLOR_PATH);
        GetManager()->AddMenuEntry((*items)[MENU_SELECT_REGION]->GetExtendedName(), MENU_SELECT_REGION);

        m_menuFirstItem = MENU_DRAW_REGIONS_CFG;
        m_menuLastItem = MENU_SELECT_REGION;

        return m_menu;
    }
}
