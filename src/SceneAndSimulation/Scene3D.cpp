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
#include "Scene3D.hpp"

namespace Antipatrea
{
    
void Scene3D::AddBoundaries(const double thick, const double h)
{
    m_tmeshObstaclesCollision.AddBoundaries(m_grid.GetMin()[0],
				       m_grid.GetMin()[1],
				       0.0,
				       m_grid.GetMax()[0],
				       m_grid.GetMax()[1],
				       h,
				       thick);
    
   m_tmeshObstaclesDraw.AddBoundaries(m_grid.GetMin()[0],
				       m_grid.GetMin()[1],
				       0.0,
				       m_grid.GetMax()[0],
				       m_grid.GetMax()[1],
				       h,
				       thick);
}

    void Scene3D::SampleValidBoxCenter(const double dims[], double c[])
    {
	 TriMeshDefault tmesh;
	 auto x = 0.5 * dims[0];
	 auto y = 0.5 * dims[1];
	 auto z = 0.5 * dims[2];	 
	 const double box[] = {-x, -y, -z,  x, y, z};
	 
	 do
	 {
	     for(int j = m_grid.GetNrDims() - 1; j >= 0; --j)
		 c[j] = RandomUniformReal(m_grid.GetMin()[j], m_grid.GetMax()[j]);    
	     tmesh.Clear();
	     tmesh.AddBox(box[0] + c[0], box[1] + c[1],  box[2] + c[2], box[3] + c[0], box[4] + c[1], box[5] + c[2]);
	 }
	 while (m_tmeshObstaclesCollision.Collision(NULL, NULL,  &tmesh, NULL, NULL) == true);
    }
    

}
