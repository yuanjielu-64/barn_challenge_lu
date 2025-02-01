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

#ifndef Antipatrea__Scene2D_HPP_
#define Antipatrea__Scene2D_HPP_

#include "Scene.hpp"
#include "Robot/Jackal.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Polygon2D.hpp"
#include <ostream>

namespace Antipatrea {
    class Scene2D : public Scene {
    public:
        Scene2D(void);

        virtual ~Scene2D(void) {
            DeleteItems<Polygon2D *>(m_obstacles);
        }

        virtual void SetupFromParams(Params &params, Robot_config &robot);

        virtual int GetNrObstacles(void) const {
            return m_obstacles.size();
        }

        virtual const Polygon2D *GetObstacle(const int i) const {
            return m_obstacles[i];
        }

        virtual Polygon2D *GetObstacle(const int i) {
            return m_obstacles[i];
        }

        virtual void AddObstacle(Polygon2D *const obs);

        virtual void thread_AddObstacle(int start, int end, std::vector<Polygon2D *> &polys);

        virtual void AddBoundaries(const double thick, const double h);

        virtual void
        Triangulate(std::vector<double> &triVertices, std::vector<int> &triIndices, std::vector<int> &triNeighs,
                    const double triAvgArea);

        virtual void DrawObstaclesPolygons(void);

        virtual void PrintObstacles(std::ostream &out) {
            PrintPolygons2D(out, m_obstacles);
        }

        virtual void SampleValidBoxCenter(const double dims[], double c[]);


    protected:
        std::vector<Polygon2D *> m_obstacles;
        double m_heightObstacle{};
        std::mutex mtx;
    };
}

#endif
