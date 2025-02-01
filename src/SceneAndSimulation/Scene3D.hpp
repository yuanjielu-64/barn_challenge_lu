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

#ifndef Antipatrea__Scene3D_HPP_
#define Antipatrea__Scene3D_HPP_

#include "Scene.hpp"

namespace Antipatrea
{
class Scene3D : public Scene
{
  public:
    Scene3D(void) : Scene()
    {
    }
    

    virtual ~Scene3D(void)
    {
    }

    virtual void AddBoundaries(const double thick, const double h);

    virtual void SampleValidBoxCenter(const double dims[], double c[]);
    
 
};
}

#endif
