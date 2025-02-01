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
#include "Programs/Setup.hpp"
#include "Components/DecompositionGrid.hpp"
#include "Components/DecompositionPRM.hpp"
#include "Components/DecompositionTriangles.hpp"
#include "Components/GroupSelectorMaxWeight.hpp"
#include "Components/GroupSelectorProbabilityUniform.hpp"
#include "Components/GroupSelectorProbabilityWeights.hpp"
#include "SceneAndSimulation/Scene2D.hpp"
#include "SceneAndSimulation/Scene3D.hpp"
#include "SceneAndSimulation/SimulatorCarTrailers.hpp"
#include "SceneAndSimulation/SimulatorBlimp.hpp"
#include "SceneAndSimulation/SimulatorRobot.hpp"
#include "Robot/Jackal.hpp"
#include "Components/TourGeneratorBNB.hpp"
#include "Components/TourGeneratorMC.hpp"
#include "Components/TourGeneratorOptimal.hpp"
#include "Components/TourGeneratorRandom.hpp"
#include "Components/TourGeneratorPDDL.hpp"
#include "Components/TourGeneratorLKH.hpp"
#include "Components/TourGeneratorExact.hpp"
#include "Planners/Juve.hpp"
#include "Planners/DDP.hpp"
#include "Planners/DWAPlanner.hpp"
#include "Planners/DDPDWAPlanner.hpp"
#include "Planners/MPPIPlanner.hpp"
#include "Planners/DDPMPPIPlanner.hpp"
#include "Planners/Sequential.hpp"
#include "Planners/RRT.hpp"
#include "Programs/Constants.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"

namespace Antipatrea {
    Setup::Setup(void)
        : SceneContainer(), SimulatorContainer(), DecompositionContainer(), ProblemContainer(),
          GroupSelectorContainer(), TourGeneratorContainer(), MPContainer() {
        double *col = GetColor(COLOR_OBSTACLES);
        col[0] = 1.0;
        col[1] = 0.3;
        col[2] = 0.7;

        col = GetColor(COLOR_DECOMPOSITION_REGIONS_CFG);
        col[0] = 0.4;
        col[1] = 0.0;
        col[2] = 0.7;

        col = GetColor(COLOR_DECOMPOSITION_REGIONS_BORDER);
        col[0] = 0.2;
        col[1] = 0.2;
        col[2] = 0.2;

        col = GetColor(COLOR_DECOMPOSITION_REGIONS_INSIDE);
        col[0] = 0.7;
        col[1] = 0.7;
        col[2] = 0.5;

        col = GetColor(COLOR_DECOMPOSITION_SELECT_REGION);
        col[0] = 0.9;
        col[1] = 0.2;
        col[2] = 0.9;

        col = GetColor(COLOR_DECOMPOSITION_REGIONS_EDGES);
        col[0] = 0.7;
        col[1] = 0.7;
        col[2] = 0.2;

        col = GetColor(COLOR_STEER);
        col[0] = 0.0;
        col[1] = 1.0;
        col[2] = 0.0;

        col = GetColor(COLOR_GOALS_REGIONS_CFG);
        col[0] = 0.4;
        col[1] = 0.7;
        col[2] = 0.4;

        col = GetColor(COLOR_GOALS_REGIONS_BORDER);
        col[0] = 0.2;
        col[1] = 0.2;
        col[2] = 0.2;

        col = GetColor(COLOR_GOALS_REGIONS_INSIDE);
        col[0] = 0.7;
        col[1] = 0.5;
        col[2] = 0.7;

        col = GetColor(COLOR_GOALS_REGIONS_EDGES);
        col[0] = 0.7;
        col[1] = 0.2;
        col[2] = 0.7;

        col = GetColor(COLOR_PATH_TO_GOAL);
        col[0] = 0.4;
        col[1] = 0.2;
        col[2] = 0.4;

        col = GetColor(COLOR_MP);
        col[0] = 0.7;
        col[1] = 0.5;
        col[2] = 0.7;

        col = GetColor(COLOR_SOLUTION);
        col[0] = 0.8;
        col[1] = 0.2;
        col[2] = 0.4;
    }

    void Setup::SetupFromParams(Params &p, Robot_config &robot) {
        const char *sceneParams = p.GetValue(Constants::KW_UseScene);
        const char *simParams = p.GetValue(Constants::KW_UseSimulator);
        const char *probParams = p.GetValue(Constants::KW_UseProblem);
        const char *decompParams = p.GetValue(Constants::KW_UseDecomposition);
        const char *mpParams = p.GetValue(Constants::KW_UseMP);
        const char *groupSelectorParams = p.GetValue(Constants::KW_UseGroupSelector);
        const char *tourGenParams = p.GetValue(Constants::KW_UseTourGenerator);

        Params::Data *data = NULL;
        DDP *ddp;
        Juve *juve;
        DWAPlanner *dwa;
        DDPDWAPlanner *ddpdwa;
        MPPIPlanner *mppi;
        DDPMPPIPlanner *ddpmppi;
        Sequential *sequential;
        RRT *rrt;

        // create instances
        if (sceneParams) {
            if (StrSameContent(sceneParams, Constants::KW_Scene2D))
                SetScene(new Scene2D());
            else if (StrSameContent(sceneParams, Constants::KW_Scene3D))
                SetScene(new Scene3D());
            else
                Logger::m_out << "Setup::SetupFromParams...warning unknown <" << Constants::KW_UseScene << " "
                        << sceneParams << std::endl;
        }

        if (simParams) {
            if (StrSameContent(simParams, Constants::KW_SimulatorCarTrailers))
                SetSimulator(new SimulatorCarTrailers());
            else if (StrSameContent(simParams, Constants::KW_SimulatorBlimp))
                SetSimulator(new SimulatorBlimp());
            else if (StrSameContent(simParams, Constants::KW_SimulatorRobot))
                SetSimulator(new SimulatorRobot());
            else
                Logger::m_out << "Setup::SetupFromParams...warning unknown <" << Constants::KW_UseSimulator << " "
                        << simParams << std::endl;
        }

        if (decompParams) {
            if (StrSameContent(decompParams, Constants::KW_DecompositionGrid))
                SetDecomposition(new DecompositionGrid());
            else if (StrSameContent(decompParams, Constants::KW_DecompositionTriangles))
                SetDecomposition(new DecompositionTriangles());
            else if (StrSameContent(decompParams, Constants::KW_DecompositionPRM))
                SetDecomposition(new DecompositionPRM());
            else
                Logger::m_out << "Setup::SetupFromParams...warning unknown <" << Constants::KW_UseDecomposition << " "
                        << decompParams << std::endl;
        }

        if (probParams) {
            if (StrSameContent(probParams, Constants::KW_Problem))
                SetProblem(new Problem());
            else
                Logger::m_out << "Setup::SetupFromParams...warning unknown <" << Constants::KW_UseProblem << " "
                        << probParams << std::endl;
        }

        if (mpParams) {
            if (StrSameContent(mpParams, Constants::KW_DDP))
                SetMP(new DDP());
            else if (StrSameContent(mpParams, Constants::KW_DWAPlanner))
                SetMP(new DWAPlanner());
            else if (StrSameContent(mpParams, Constants::KW_DDPDWAPlanner))
                SetMP(new DDPDWAPlanner());
            else if (StrSameContent(mpParams, Constants::KW_MPPIPlanner))
                SetMP(new MPPIPlanner());
            else if (StrSameContent(mpParams, Constants::KW_DDPMPPIPlanner))
                SetMP(new DDPMPPIPlanner());
            else if (StrSameContent(mpParams, Constants::KW_Sequential))
                SetMP(new Sequential());
            else if (StrSameContent(mpParams, Constants::KW_Juve))
                SetMP(new Juve());
            else if (StrSameContent(mpParams, Constants::KW_RRT))
                SetMP(new RRT());
            else
                Logger::m_out << "Setup::SetupFromParams...warning unknown <" << Constants::KW_UseMP << " " << mpParams
                        << std::endl;
        }

        if (groupSelectorParams) {
            if (StrSameContent(groupSelectorParams, Constants::KW_GroupSelectorProbabilityUniform))
                SetGroupSelector(new GroupSelectorProbabilityUniform());
            else if (StrSameContent(groupSelectorParams, Constants::KW_GroupSelectorProbabilityWeights))
                SetGroupSelector(new GroupSelectorProbabilityWeights());
            else if (StrSameContent(groupSelectorParams, Constants::KW_GroupSelectorMaxWeight))
                SetGroupSelector(new GroupSelectorMaxWeight());
            else
                Logger::m_out << "Setup::SetupFromParams...warning unknown <" << Constants::KW_UseGroupSelector << " "
                        << groupSelectorParams << std::endl;
        }

        if (tourGenParams) {
            if (StrSameContent(tourGenParams, Constants::KW_TourGeneratorRandom)) {
                SetTourGenerator(new TourGeneratorRandom());
                Logger::m_out << "...using Random tour genreator" << std::endl;
            } else if (StrSameContent(tourGenParams, Constants::KW_TourGeneratorExactNormalOrder)) {
                SetTourGenerator(new TourGeneratorExact(true));
                Logger::m_out << "...using ExactNormalOrder tour genreator" << std::endl;
            } else if (StrSameContent(tourGenParams, Constants::KW_TourGeneratorExactReverseOrder)) {
                SetTourGenerator(new TourGeneratorExact(false));
                Logger::m_out << "...using ExactReverseOrder tour genreator" << std::endl;
            } else if (StrSameContent(tourGenParams, Constants::KW_TourGeneratorOptimal))
                SetTourGenerator(new TourGeneratorOptimal());
            else if (StrSameContent(tourGenParams, Constants::KW_TourGeneratorBNB))
                SetTourGenerator(new TourGeneratorBNB());
            else if (StrSameContent(tourGenParams, Constants::KW_TourGeneratorMC))
                SetTourGenerator(new TourGeneratorMC());
            else if (StrSameContent(tourGenParams, Constants::KW_TourGeneratorPDDL))
                SetTourGenerator(new TourGeneratorPDDL());
            else if (StrSameContent(tourGenParams, Constants::KW_TourGeneratorLKH))
                SetTourGenerator(new TourGeneratorLKH());
            else
                Logger::m_out << "Setup::SetupFromParams...warning unknown <" << Constants::KW_UseTourGenerator << " "
                        << tourGenParams << std::endl;
        }

        // setup pointers
        if (GetSimulator()) {
            GetSimulator()->SetRobot(&robot);
            GetSimulator()->SetScene(GetScene());
        }
        if (GetDecomposition()) {
            GetDecomposition()->SetSimulator(GetSimulator());
            GetDecomposition()->SetProblem(GetProblem());
        }

        if (GetMP()) {
            GetMP()->SetSimulator(GetSimulator());
            GetMP()->SetProblem(GetProblem());
            GetMP()->SetDecomposition(GetDecomposition());

            if ((ddp = dynamic_cast<DDP *>(GetMP()))) {
                ddp->SetGroupSelector(GetGroupSelector());
            } else if ((dwa = dynamic_cast<DWAPlanner *>(GetMP()))) {
                dwa->SetGroupSelector(GetGroupSelector());
            } else if ((ddpdwa = dynamic_cast<DDPDWAPlanner *>(GetMP()))) {
                ddpdwa->SetGroupSelector(GetGroupSelector());
            } else if ((mppi = dynamic_cast<MPPIPlanner *>(GetMP()))) {
                mppi->SetGroupSelector(GetGroupSelector());
            } else if ((ddpmppi = dynamic_cast<DDPMPPIPlanner *>(GetMP()))) {
                ddpmppi->SetGroupSelector(GetGroupSelector());
            }  else if ((sequential = dynamic_cast<Sequential *>(GetMP()))) {
                sequential->SetGroupSelector(GetGroupSelector());
            }  else if ((rrt = dynamic_cast<RRT *>(GetMP()))) {
                rrt->SetGroupSelector(GetGroupSelector());
                rrt->SetTourGenerator(GetTourGenerator());
            } else if ((juve = dynamic_cast<Juve *>(GetMP()))) {
                juve->SetGroupSelector(GetGroupSelector());
                juve->SetTourGenerator(GetTourGenerator());
            }
        }

        // setup from params
        if (GetScene() && sceneParams && (data = p.GetData(sceneParams)) && data->m_params)
            GetScene()->SetupFromParams(*(data->m_params), *GetSimulator()->GetRobot());
        if (GetSimulator() && simParams && (data = p.GetData(simParams)) && data->m_params)
            GetSimulator()->SetupFromParams(*(data->m_params));
        if (GetDecomposition() && decompParams && (data = p.GetData(decompParams)) && data->m_params)
            GetDecomposition()->SetupFromParams(*(data->m_params));
        if (!(*GetSimulator()->GetRobot()).getGlobalGoalCfg().empty())
            GetProblem()->SetupGoalFromPath(*GetSimulator()->GetRobot());
        if (GetGroupSelector() && groupSelectorParams && (data = p.GetData(groupSelectorParams)) && data->m_params)
            GetGroupSelector()->SetupFromParams(*(data->m_params));
        if (GetMP() && mpParams && (data = p.GetData(mpParams)) && data->m_params)
            GetMP()->SetupFromParams(*(data->m_params));

        // finish what is needed to get ready for motion planning
        if (GetSimulator()) {
            GetSimulator()->SetVelocityScaleConversion(p.GetValueAsDouble(Constants::KW_VelocityScaleConversion,
                                                                          GetSimulator()->
                                                                          GetVelocityScaleConversion()));

            GetSimulator()->CompleteSetup();
            if (simParams && (data = p.GetData(simParams)) && data->m_params)
                RunInitialState(*(data->m_params));

            if (GetProblem() && probParams && (data = p.GetData(probParams)) && data->m_params)
                RunInitialState(*(data->m_params));

            GetProblem()->m_gShowIds.resize(GetProblem()->GetGoals()->size());
            for (int i = 0; i < GetProblem()->m_gShowIds.size(); ++i) {
                GetProblem()->m_gShowIds[i] = i;
            }
        }

        RunAdjustTimeBounds(p);

        // setup graphics
        data = p.GetData(Constants::KW_Graphics);
        if (data && data->m_params) {
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorObstacles, GetColor(COLOR_OBSTACLES),
                                               NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorDecompositionRegionsCfg,
                                               GetColor(COLOR_DECOMPOSITION_REGIONS_CFG), NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorDecompositionRegionsInside,
                                               GetColor(COLOR_DECOMPOSITION_REGIONS_INSIDE), NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorDecompositionRegionsBorder,
                                               GetColor(COLOR_DECOMPOSITION_REGIONS_BORDER), NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorDecompositionRegionsEdges,
                                               GetColor(COLOR_DECOMPOSITION_REGIONS_EDGES), NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorDecompositionSelectRegion,
                                               GetColor(COLOR_DECOMPOSITION_SELECT_REGION), NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorSteer, GetColor(COLOR_STEER), NR_COLOR_COMPONENTS);

            data->m_params->GetValuesAsDoubles(Constants::KW_ColorGoalsRegionsCfg, GetColor(COLOR_GOALS_REGIONS_CFG),
                                               NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorGoalsRegionsInside,
                                               GetColor(COLOR_GOALS_REGIONS_INSIDE), NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorGoalsRegionsBorder,
                                               GetColor(COLOR_GOALS_REGIONS_BORDER), NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorGoalsRegionsEdges,
                                               GetColor(COLOR_GOALS_REGIONS_EDGES), NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorPathToGoal, GetColor(COLOR_PATH_TO_GOAL),
                                               NR_COLOR_COMPONENTS);

            data->m_params->GetValuesAsDoubles(Constants::KW_ColorMP, GetColor(COLOR_MP), NR_COLOR_COMPONENTS);
            data->m_params->GetValuesAsDoubles(Constants::KW_ColorSolution, GetColor(COLOR_SOLUTION),
                                               NR_COLOR_COMPONENTS);
        }
    }

    void Setup::UpdateFromParams(Antipatrea::Params &p, Robot_config &robot, int &robot_state) {
        Timer::Clock aaa;
        Timer::Start(aaa);

        const char *sceneParams = p.GetValue(Constants::KW_UseScene);
        const char *simParams = p.GetValue(Constants::KW_UseSimulator);
        const char *mpParams = p.GetValue(Constants::KW_UseMP);
        const char *decompParams = p.GetValue(Constants::KW_UseDecomposition);

        if (sceneParams) {
            if (StrSameContent(sceneParams, Constants::KW_Scene2D))
                SetScene(new Scene2D());
        }

        if (decompParams) {
            if (StrSameContent(decompParams, Constants::KW_DecompositionGrid))
                SetDecomposition(new DecompositionGrid());
            else if (StrSameContent(decompParams, Constants::KW_DecompositionPRM))
                SetDecomposition(new DecompositionPRM());
        }

        if (mpParams) {
            if (StrSameContent(mpParams, Constants::KW_DDP))
                SetMP(new DDP());
            else if (StrSameContent(mpParams, Constants::KW_DWAPlanner))
                SetMP(new DWAPlanner());
            else if (StrSameContent(mpParams, Constants::KW_DDPDWAPlanner))
                SetMP(new DDPDWAPlanner());
            else if (StrSameContent(mpParams, Constants::KW_MPPIPlanner))
                SetMP(new MPPIPlanner());
            else if (StrSameContent(mpParams, Constants::KW_DDPMPPIPlanner))
                SetMP(new DDPMPPIPlanner());
            else if (StrSameContent(mpParams, Constants::KW_Sequential))
                SetMP(new Sequential());
            else if (StrSameContent(mpParams, Constants::KW_Juve))
                SetMP(new Juve());
            else if (StrSameContent(mpParams, Constants::KW_RRT))
                SetMP(new RRT());
            else
                Logger::m_out << "Setup::SetupFromParams...warning unknown <" << Constants::KW_UseMP << " " << mpParams
                        << std::endl;
        }

        if (GetSimulator() && robot_state != robot.getRobotState()) {
            Params::Data *data = NULL;
            data = p.GetData(simParams);
            GetSimulator()->SetupFromParams(*(data->m_params), robot);
            GetSimulator()->CompleteSetup();
            robot_state = robot.getRobotState();
        }

        if (GetSimulator())
            GetSimulator()->SetScene(GetScene());

        if (GetDecomposition()) {
            GetDecomposition()->SetSimulator(GetSimulator());
            GetDecomposition()->SetProblem(GetProblem());
        }

        if (GetMP()) {
            GetMP()->SetSimulator(GetSimulator());
            GetMP()->SetProblem(GetProblem());
            GetMP()->SetDecomposition(GetDecomposition());
        }

        Params::Data *data = NULL;
        //if (GetScene() && sceneParams && (data = p.GetData(sceneParams)) && data->m_params)
        //    GetScene()->SetupFromParams(*(p.GetData(sceneParams)->m_params), *GetSimulator()->GetRobot());

        if (GetDecomposition() && decompParams && (data = p.GetData(decompParams)) && data->m_params)
            GetDecomposition()->SetupFromParams(*(data->m_params));

        if (GetMP() && mpParams && (data = p.GetData(mpParams)) && data->m_params)
            GetMP()->SetupFromParams(*(data->m_params));

        if (!(*GetSimulator()->GetRobot()).getGlobalGoalCfg().empty())
            GetProblem()->SetupGoalFromPath(*GetSimulator()->GetRobot());

        GetSimulator()->CompleteSetup();
        if (simParams && (p.GetData(simParams)) && p.GetData(simParams)->m_params)
            RunInitialState(*(p.GetData(simParams)->m_params));

        GetProblem()->m_gShowIds.resize(GetProblem()->GetGoals()->size());
        for (int i = 0; i < GetProblem()->m_gShowIds.size(); ++i)
            GetProblem()->m_gShowIds[i] = i;

        RunAdjustTimeBounds(p);
    }


    void Setup::RunInitialState(Params &p) {
        auto dataInit = p.GetData(Constants::KW_CurrentState);
        if (dataInit && dataInit->m_values.size() > 0) {
            auto currs = GetSimulator()->GetStateAllocator()->New();
            GetSimulator()->GetState(currs);
            p.GetValuesAsDoubles(Constants::KW_CurrentState, currs, GetSimulator()->GetStateAllocator()->GetDim());

            currs[3] = (*GetSimulator()->GetRobot()).getVelocity();
            currs[4] = (*GetSimulator()->GetRobot()).getAngularVelocity();

            GetSimulator()->SetState(currs);
            //Logger::m_out << " current state: ";
            //GetSimulator()->GetStateAllocator()->Print(Logger::m_out, currs);
            GetSimulator()->GetStateAllocator()->Delete(currs);
        }
    }

    void Setup::RunConstructDecomposition(Params &p) {
        if (GetDecomposition() == NULL)
            return;

        double tmax = Constants::MAX_RUNTIME_DECOMPOSITION_CONSTRUCT;
        const char *decompParams = p.GetValue(Constants::KW_UseDecomposition);
        Params::Data *data = NULL;

        if (decompParams && (data = p.GetData(decompParams)) && data->m_params)
            tmax = data->m_params->GetValueAsDouble(Constants::KW_Runtime, tmax);

        Timer::Clock clk;
        Timer::Start(clk);
        GetDecomposition()->Construct(tmax);

        Stats::GetSingleton()->AddValue("TimeDecomposition", Timer::Elapsed(clk));
        GetDecomposition()->SetRegionColorsBasedOnClearance();
    }

    void Setup::RunAdjustTimeBounds(Params &p) {
        const double adjust = p.GetValueAsDouble(Constants::KW_AdjustTimeBounds, -1.0);
        const double scale = 1.0;

        auto bounds = &(GetProblem()->GetTimeBounds()->operator[](0));

        //Logger::m_out << "AdjustTimeBounds = " << adjust << std::endl;

        for (int i = GetProblem()->GetGoals()->size() - 1; i >= 0; --i) {
            bounds[2 * i] *= scale;
            bounds[2 * i + 1] *= scale;

            if (adjust < 0) //hack: infinity
            {
                bounds[2 * i] = 0.0;
                bounds[2 * i + 1] = 100000.0; //inf
            } else if (bounds[2 * i + 1] != INFINITY) {
                const double val = 0.5 * (bounds[2 * i] + bounds[2 * i + 1]) * adjust;
                bounds[2 * i] -= val;
                if (bounds[2 * i] < 0)
                    bounds[2 * i] = 0.0;
                bounds[2 * i + 1] += val;
            }
        }
    }
}
