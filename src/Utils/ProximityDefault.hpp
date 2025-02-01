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

#ifndef Antipatrea__ProximityDefault_HPP_
#define Antipatrea__ProximityDefault_HPP_

#include "Utils/ProximityGNAT.hpp"

namespace Antipatrea
{
    template <typename Key, typename DistFnData>
    class ProximityDefault : public ProximityGNAT<Key, DistFnData>
    { 
    public:
	ProximityDefault(void) : ProximityGNAT<Key, DistFnData>()
	{
	}
	
	
	virtual ~ProximityDefault(void)
	{
	}
    };
}

#endif








