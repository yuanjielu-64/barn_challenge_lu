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

#ifndef Antipatrea__Scene_HPP_
#define Antipatrea__Scene_HPP_

#include "Components/Component.hpp"
#include "Robot/Jackal.hpp"
#include "Utils/Grid.hpp"
#include "Utils/Params.hpp"
#include "Utils/TriMeshDefault.hpp"

#include <vector>

namespace Antipatrea {
    class Scene : public Component {
    public:
        Scene(void) : Component() {
        }

        virtual ~Scene(void) {
        }

        virtual void SetupFromParams(Params &params, Robot_config &robot);

        virtual Grid *GetGrid(void) {
            return &m_grid;
        }

        virtual TriMesh *GetObstaclesCollisionMesh(void) {
            return &m_tmeshObstaclesCollision;
        }

        virtual TriMesh *GetObstaclesDrawMesh(void) {
            return &m_tmeshObstaclesDraw;
        }

        virtual TriMesh *GetTerrainMesh(void) {
            return &m_tmeshTerrain;
        }

        virtual void AdjustGrid(const double eps);

        virtual void AddBoundaries(const double thick, const double h) = 0;

        virtual void DrawObstacles(void) {
            m_tmeshObstaclesDraw.Draw();
        }

        virtual void DrawTerrain(void) {
            m_tmeshTerrain.Draw();
        }

        virtual void SampleValidBoxCenter(const double dims[], double c[]) = 0;

    protected:
        Grid m_grid;
        TriMeshDefault m_tmeshTerrain;
        TriMeshDefault m_tmeshObstaclesCollision;
        TriMesh m_tmeshObstaclesDraw;
    };

    ClassContainer(Scene, m_scene);
}

#endif
