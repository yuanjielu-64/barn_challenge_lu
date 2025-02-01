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

#ifndef Antipatrea__ProgramsConstants_HPP_
#define Antipatrea__ProgramsConstants_HPP_

namespace Antipatrea {
/**
 *@brief Keywords and default values.
 */
    namespace Constants {
        const char KW_TextureObstaclesFile[] = "TextureObstaclesFile";
        const char KW_UseScene[] = "UseScene";
        const char KW_UseSimulator[] = "UseSimulator";
        const char KW_UseDecomposition[] = "UseDecomposition";
        const char KW_UseProblem[] = "UseProblem";
        const char KW_UseMP[] = "UseMP";
        const char KW_UseMap[] = "UseMap";
        const char KW_UseGroupSelector[] = "UseGroupSelector";
        const char KW_UseGroupVertexSelector[] = "UseGroupVertexSelector";
        const char KW_UseTourGenerator[] = "UseTourGenerator";

        const char KW_ColorObstacles[] = "ColorObstacles";
        const char KW_ColorDecompositionRegionsCfg[] = "ColorDecompositionRegionsCfg";
        const char KW_ColorDecompositionRegionsBorder[] = "ColorDecompositionRegionsBorder";
        const char KW_ColorDecompositionRegionsInside[] = "ColorDecompositionRegionsInside";
        const char KW_ColorDecompositionRegionsEdges[] = "ColorDecompositionRegionsEdges";
        const char KW_ColorDecompositionSelectRegion[] = "ColorDecompositionSelectRegion";
        const char KW_ColorGoalsRegionsCfg[] = "ColorGoalsRegionsCfg";
        const char KW_ColorGoalsRegionsBorder[] = "ColorGoalsRegionsBorder";
        const char KW_ColorGoalsRegionsInside[] = "ColorGoalsRegionsInside";
        const char KW_ColorGoalsRegionsEdges[] = "ColorGoalsRegionsEdges";
        const char KW_ColorPathToGoal[] = "ColorPathToGoal";
        const char KW_ColorSteer[] = "ColorSteer";
        const char KW_ColorMP[] = "ColorMP";
        const char KW_ColorSolution[] = "ColorSolution";

        const char KW_AdjustTimeBounds[] = "AdjustTimeBounds";

        const char KW_Runtime[] = "Runtime";
    }
}

#endif
