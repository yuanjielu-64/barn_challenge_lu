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
#include "Scene.hpp"
#include "Components/Constants.hpp"
#include "Utils/TriMeshStreamer.hpp"

namespace Antipatrea {
    void Scene::SetupFromParams(Params &params, Robot_config &robot) {
        Component::SetupFromParams(params);

        auto data = params.GetData(Constants::KW_Grid);
        if (data && data->m_params)
            m_grid.SetupFromParams(*(data->m_params));

        auto name = params.GetValue(Constants::KW_ObstaclesCollisionMeshFile);
        if (name)
            TriMeshReader(name, m_tmeshObstaclesCollision);
        name = params.GetValue(Constants::KW_ObstaclesDrawMeshFile);
        if (name)
            TriMeshReader(name, m_tmeshObstaclesDraw);
        name = params.GetValue(Constants::KW_TerrainMeshFile);
        if (name)
            TriMeshReader(name, m_tmeshTerrain);

        data = params.GetData(Constants::KW_Boundaries);
        if (data && data->m_params)
            AddBoundaries(
                    data->m_params->GetValueAsDouble(Constants::KW_Thickness, Constants::SCENE_BOUNDARY_THICKNESS),
                    data->m_params->GetValueAsDouble(Constants::KW_Height, Constants::SCENE_BOUNDARY_HEIGHT));

        const double eps = params.GetValueAsDouble(Constants::KW_AdjustGrid, INFINITY);
        if (eps != INFINITY)
            AdjustGrid(eps);
    }

    void Scene::AdjustGrid(const double eps) {
        const int n = m_grid.GetNrDims();
        const int nrDimsToAdjust = 2;
        const double *gmin = m_grid.GetMin();
        const double *gmax = m_grid.GetMax();
        const double *omin = m_tmeshObstaclesCollision.GetBoundingBoxMin();
        const double *omax = m_tmeshObstaclesCollision.GetBoundingBoxMax();

        std::vector<double> pmin;
        std::vector<double> pmax;

        pmin.resize(n);
        pmax.resize(n);

        for (int i = 0; i < n && i < nrDimsToAdjust; ++i) {
            pmin[i] = omin[i] - eps;
            pmax[i] = omin[i] + eps;
            if (gmin[i] < pmin[i])
                pmin[i] = gmin[i];
            if (gmax[i] > pmax[i])
                pmax[i] = gmax[i];
        }
        for (int i = nrDimsToAdjust; i < n; ++i) {
            pmin[i] = gmin[i];
            pmax[i] = gmax[i];
        }

        m_grid.Setup(n, m_grid.GetDims(), &pmin[0], &pmax[0]);
    }
}
