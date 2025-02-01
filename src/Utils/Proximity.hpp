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

#ifndef Antipatrea__Proximity_HPP_
#define Antipatrea__Proximity_HPP_

#include "Utils/ProximityQuery.hpp"
#include "Utils/ProximityResults.hpp"
#include <vector>
#include <cstdlib>

namespace Antipatrea
{
    template <typename Key, typename DistFnData>
    class Proximity
    {
    public:
	Proximity(void)
	{
	    m_construct = false;	    
	}

	virtual ~Proximity(void) 
	{
	}

	typedef double (*DistFn) (const Key, const Key, DistFnData);

	DistFn     m_distFn;
	DistFnData m_distFnData;
	
	bool IsDataStructureConstructed(void) const
	{
	    return m_construct;
	}

	const std::vector<Key>* GetKeys(void) const
	{
	    return &m_keys;
	}

	virtual void AddKey(const Key key)
	{
	    m_keys.push_back(key);	    
	}
	
	virtual void ConstructDataStructure(void)
	{
	    m_construct = true;
	}

	virtual void ClearDataStructure(void)
	{
	    m_construct = false;
	    m_keys.clear();
	}

	virtual void Clear(void)
	{
	    ClearDataStructure();
	}

	virtual Key Neighbor(ProximityQuery<Key> & query, double * const d = NULL)
	{
	    ProximityResults<Key> pr;
	    
	    query.SetNrNeighbors(1);	    
	    Neighbors(query, pr);

	    if(pr.GetNrResults() == 0)
	    {
		if(d)
		    *d = INFINITY;
		return query.GetKey();
	    }
	    
	    if(d)
		*d = pr.GetDistance(0);	    
	    return pr.GetKey(0);
	}
	
	virtual void Neighbors(ProximityQuery<Key>   & query, 
			       ProximityResults<Key> & results) = 0;

    protected:
	bool             m_construct;
	std::vector<Key> m_keys;	
    };      
}

#endif 








