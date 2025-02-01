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

#ifndef Antipatrea__ProximityBruteForce_HPP_
#define Antipatrea__ProximityBruteForce_HPP_

#include "Utils/Proximity.hpp"

namespace Antipatrea
{
    template <typename Key, typename DistFnData>
    class ProximityBruteForce : public Proximity<Key, DistFnData>
    { 
    public:
	ProximityBruteForce(void) : Proximity<Key, DistFnData>(),
	    m_construct(true)
	{
	}
	
	
	virtual ~ProximityBruteForce(void)
	{
	}
	
	virtual void Neighbors(ProximityQuery<Key> & query, 
			       ProximityResults<Key> & results)
	{
	    if(this->IsDataStructureConstructed() == false)
		this->ConstructDataStructure();
	    
	    const Key  qkey = query.GetKey();
	    const int  size = this->m_keys.size();
	    
	    results.Clear();
	    results.SetNrNeighborsAndRange(query.GetNrNeighbors(), query.GetRange());
	    for(int i = 0; i < size; i++)
	    {
		const Key ikey = this->m_keys[i];
	    	results.Insert(ikey, this->m_distFn(qkey, ikey, m_distFnData));
	    }
	}
    };
}

#endif








