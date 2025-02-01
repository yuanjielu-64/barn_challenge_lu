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
#include "Scene2D.hpp"
#include "Components/Constants.hpp"
#include "Robot/Jackal.hpp"
#include "External/ShewchukTriangle.hpp"
#include "Utils/GDraw.hpp"
#include <algorithm>
#include "Utils/Timer.hpp"
#include <thread>

namespace Antipatrea {
    Scene2D::Scene2D() {}

    void Scene2D::SetupFromParams(Params &params, Robot_config &robot) {
        std::vector<Polygon2D *> polys;
        Timer::Clock aaa;
        Timer::Start(aaa);
        m_heightObstacle = params.GetValueAsDouble(Constants::KW_ObstacleHeight, m_heightObstacle);

        //if (!robot.polygons.empty())
        //    ReadPolygons2DFromLaser(robot.polygons, polys, robot.getRobotState());

         //if (!robot.costmapData.empty())
         //    ReadPolygons2DFromLaser(robot.costmapData, polys, 0);

        // Logger::m_out << "polygon Nr:" << robot.getDataMap().size() << std::endl;

        m_obstacles.clear();

        for (auto &poly: polys)
            AddObstacle(poly);

        const double GRID_MIN[] = {robot.grid_min[0], robot.grid_min[1], 0};
        const double GRID_MAX[] = {robot.grid_max[0], robot.grid_max[1], 0};

        m_grid.Setup(2, Constants::GRID_DIMS, GRID_MIN, GRID_MAX);
        m_heightObstacle = Constants::SCENE_OBSTACLE_HEIGHT;

        Scene::SetupFromParams(params, robot);

        m_tmeshTerrain.AddBox2D(m_grid.GetMin()[0],
                                m_grid.GetMin()[1],
                                m_grid.GetMax()[0],
                                m_grid.GetMax()[1]);

    }

    void Scene2D::AddObstacle(Polygon2D *const poly) {
        m_obstacles.push_back(poly);
        m_tmeshObstaclesCollision.AddPolygon(*poly);
        m_tmeshObstaclesDraw.AddExtrudedPolygon(*poly, 0, m_heightObstacle);
    }

    void Scene2D::thread_AddObstacle(int start, int end, std::vector<Polygon2D *> &polys) {
        for (int i = start; i < end; i++) {
            Polygon2D* poly = polys[i];
            {
                std::lock_guard<std::mutex> guard(mtx);
                m_obstacles.push_back(poly);
                m_tmeshObstaclesCollision.AddPolygon(*poly);
                m_tmeshObstaclesDraw.AddExtrudedPolygon(*poly, 0, m_heightObstacle);
            }
        }
    }

    void Scene2D::AddBoundaries(const double thick, const double h) {
        m_tmeshObstaclesCollision.AddBoundaries2D(m_grid.GetMin()[0], m_grid.GetMin()[1], m_grid.GetMax()[0],
                                                  m_grid.GetMax()[1], thick);
        m_tmeshObstaclesDraw.AddBoundaries(m_grid.GetMin()[0], m_grid.GetMin()[1], 0.0, m_grid.GetMax()[0],
                                           m_grid.GetMax()[1], h, thick);
    }

    void
    Scene2D::Triangulate(std::vector<double> &triVertices, std::vector<int> &triIndices, std::vector<int> &triNeighs,
                         const double triAvgArea) {
        std::vector<double> vertices;
        std::vector<int> nrVerticesPerContour;
        std::vector<double> ptsInsideHoles;
        const int nrObsts = GetNrObstacles();
        Polygon2D *poly;

        const double *pmin = GetGrid()->GetMin();
        const double *pmax = GetGrid()->GetMax();

        // grid boundaries
        vertices.push_back(pmin[0]);
        vertices.push_back(pmin[1]);
        vertices.push_back(pmax[0]);
        vertices.push_back(pmin[1]);
        vertices.push_back(pmax[0]);
        vertices.push_back(pmax[1]);
        vertices.push_back(pmin[0]);
        vertices.push_back(pmax[1]);
        nrVerticesPerContour.push_back(4);

        // obstacles as holes
        ptsInsideHoles.resize(2 * nrObsts);
        for (int i = 0; i < nrObsts; ++i) {
            poly = GetObstacle(i);
            nrVerticesPerContour.push_back(poly->GetNrVertices());
            poly->GetRepresentativePoint(&ptsInsideHoles[2 * i]);

            vertices.insert(vertices.end(), poly->GetVertices()->begin(), poly->GetVertices()->end());
        }

        Logger::m_out << "calling triangulation" << std::endl;

        TriangulatePolygonWithHoles2D(false, -1, triAvgArea, vertices.size() / 2, &vertices[0],
                                      &nrVerticesPerContour[0], ptsInsideHoles.size() / 2,
                                      &ptsInsideHoles[0], &triVertices, &triIndices, &triNeighs);

        Logger::m_out << "done" << std::endl;
    }


    void Scene2D::DrawObstaclesPolygons(void) {
        for (int i = m_obstacles.size() - 1; i >= 0; --i)
            GDrawPolygon2D(*(m_obstacles[i]));
    }

    void Scene2D::SampleValidBoxCenter(const double dims[], double c[]) {
        TriMeshDefault tmesh;
        auto x = 0.5 * dims[0];
        auto y = 0.5 * dims[1];
        const double box[] = {-x, -y, x, y};

        do {
            for (int j = m_grid.GetNrDims() - 1; j >= 0; --j)
                c[j] = RandomUniformReal(m_grid.GetMin()[j], m_grid.GetMax()[j]);
            tmesh.Clear();
            tmesh.AddBox2D(box[0] + c[0], box[1] + c[1], box[2] + c[0], box[3] + c[1]);
        } while (m_tmeshObstaclesCollision.Collision(NULL, NULL, &tmesh, NULL, NULL) == true);
    }


}
