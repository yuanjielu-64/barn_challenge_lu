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
#include "Programs/GManagerGoals.hpp"
#include "Utils/GManager.hpp"
#include "Components/RegionGeometric.hpp"
#include "Components/DecompositionGeometric.hpp"
#include "Programs/GManagerDecomposition.hpp"
#include <algorithm>

namespace Antipatrea {
    void GManagerGoals::HandleEventOnDisplay(void) {
        GManagerComponent::HandleEventOnDisplay();

        auto items = GetManager()->GetMenuItems();
        auto decomp = GetSetup()->GetDecomposition();
        auto sim = GetSetup()->GetSimulator();
        auto prob = GetSetup()->GetProblem();

        if (decomp == NULL || sim == NULL || prob == NULL)
            return;

        if (HasAllFlags((*items)[MENU_DRAW_GOALS_CFG]->GetFlags(), GMenuItem::FLAG_ON))
            decomp->DrawRegionsCfg(Region::STATUS_FREE | Region::STATUS_GOAL);

        if (HasAllFlags((*items)[MENU_DRAW_GOALS_INSIDE]->GetFlags(), GMenuItem::FLAG_ON))
            prob->DrawGoals();

        if (HasAllFlags((*items)[MENU_DRAW_GOALS_BORDER]->GetFlags(), GMenuItem::FLAG_ON)) {
            GDrawWireframe(true);
            prob->DrawGoals();
            GDrawWireframe(false);
        }

        if (HasAllFlags((*items)[MENU_DRAW_GOALS_EDGES]->GetFlags(), GMenuItem::FLAG_ON)) {
            GDrawColor(GetSetup()->GetColor(Setup::COLOR_GOALS_REGIONS_EDGES));
            decomp->DrawEdges(DecompositionEdge::STATUS_GOAL);
        }
    }

    bool GManagerGoals::HandleEventOnMenu(const int item) {
        if (item == MENU_DRAW_GOALS_COLOR_CLEARANCE) {
            GetSetup()->GetDecomposition()->SetRegionColorsBasedOnClearance(Region::STATUS_FREE | Region::STATUS_GOAL);
            return true;
        } else if (item == MENU_DRAW_GOALS_COLOR_PATH) {
            if (GetSetup()->GetProblem()->GetGoals()->size() > 0) {
                int gid = 0;
                auto curr = std::find_if(GetManager()->GetComponents()->begin(),
                                         GetManager()->GetComponents()->end(),
                                         [](GManagerComponent *comp) {
                                             return dynamic_cast<GManagerDecomposition *>(comp) != NULL;
                                         });
                if (curr != GetManager()->GetComponents()->end())
                    gid = dynamic_cast<GManagerDecomposition *>(*curr)->GetSelectedGoal();

                GetSetup()->GetDecomposition()->SetRegionColorsBasedOnPathCost(gid, Region::STATUS_FREE |
                                                                                    Region::STATUS_GOAL);
            }
            return true;
        } else if (item == MENU_DRAW_GOALS_COLOR_SAME) {
            GetSetup()->GetDecomposition()->SetRegionColors(
                    GetSetup()->GetColor(Setup::COLOR_DECOMPOSITION_REGIONS_INSIDE),
                    Region::STATUS_FREE | Region::STATUS_GOAL);
            return true;
        } else
            return GManagerComponent::HandleEventOnMenu(item);
    }

    int GManagerGoals::PrepareMenu(void) {
        GManagerComponent::PrepareMenu();

        auto items = GetManager()->GetMenuItems();

        MENU_DRAW_GOALS_CFG = items->size();
        items->push_back(new GMenuItem("draw goals cfg", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_GOALS_BORDER = items->size();
        items->push_back(new GMenuItem("draw goals border", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_GOALS_INSIDE = items->size();
        items->push_back(new GMenuItem("draw goals inside", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_GOALS_EDGES = items->size();
        items->push_back(new GMenuItem("draw goal edges", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_GOALS_COLOR_SAME = items->size();
        items->push_back(new GMenuItem("goals color same", GMenuItem::FLAG_OFF));
        MENU_DRAW_GOALS_COLOR_CLEARANCE = items->size();
        items->push_back(new GMenuItem("goals color clearance", GMenuItem::FLAG_OFF));
        MENU_DRAW_GOALS_COLOR_PATH = items->size();
        items->push_back(new GMenuItem("goals color path", GMenuItem::FLAG_OFF));

        GetManager()->AddMenuEntry((*items)[MENU_DRAW_GOALS_CFG]->GetExtendedName(), MENU_DRAW_GOALS_CFG);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_GOALS_BORDER]->GetExtendedName(), MENU_DRAW_GOALS_BORDER);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_GOALS_INSIDE]->GetExtendedName(), MENU_DRAW_GOALS_INSIDE);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_GOALS_EDGES]->GetExtendedName(), MENU_DRAW_GOALS_EDGES);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_GOALS_COLOR_SAME]->GetExtendedName(), MENU_DRAW_GOALS_COLOR_SAME);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_GOALS_COLOR_CLEARANCE]->GetExtendedName(),
                                   MENU_DRAW_GOALS_COLOR_CLEARANCE);
        GetManager()->AddMenuEntry((*items)[MENU_DRAW_GOALS_COLOR_PATH]->GetExtendedName(), MENU_DRAW_GOALS_COLOR_PATH);

        m_menuFirstItem = MENU_DRAW_GOALS_CFG;
        m_menuLastItem = MENU_DRAW_GOALS_COLOR_PATH;

        return m_menu;
    }
}
