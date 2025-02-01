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

#ifndef Antipatrea__ProximityResults_HPP_
#define Antipatrea__ProximityResults_HPP_

#include "Utils/BinarySearch.hpp"
#include <vector>

namespace Antipatrea
{
    template <typename Key>
    class ProximityResults
    {
    public:
	ProximityResults(void)
	{
	    m_nrKeysInserted = 0;
	}
	
	~ProximityResults(void) 
	{
	}
	
	void SetNrNeighborsAndRange(const int n, const double range)
	{
	    const int old_size = m_keys.size();
	    m_keys.resize(n);
	    m_dists.resize(n);
	    for(int i = old_size; i < n; i++)
		m_dists[i]  = range;
	}
	
	double GetMaxDistance(void) const
	{
	    return m_dists[m_dists.size() - 1];
	}	
	
	int GetSize(void) const
	{
	    return m_keys.size();
	}
	
	int GetNrResults(void) const
	{
	    const int n = m_keys.size();
	    return m_nrKeysInserted < n ? m_nrKeysInserted : n;
	}
	
	Key GetKey(const int i) const
	{
	    return m_keys[i];
	}
	
	double GetDistance(const int i) const
	{
	    return m_dists[i];
	}
	
	void Insert(const Key key, const double d)
	{	
	    const int size = m_keys.size();
	    
	    if(d >= m_dists[size - 1])
		return;
	    
	    const int pos = BinarySearch::SortedPosition<double>(&(m_dists[0]), d, 0, 
								 m_nrKeysInserted < size ? 
								 m_nrKeysInserted - 1 : size - 1);
	    
	    if(pos < size)
	    {
		m_keys.insert(m_keys.begin() + pos, key);
		m_dists.insert(m_dists.begin() + pos, d);
		
		m_keys.pop_back();
		m_dists.pop_back();
		
		++m_nrKeysInserted;
	    }	    
	}
	
	void Clear(void)
	{
	    m_nrKeysInserted = 0;
	    m_keys.clear();
	    m_dists.clear();
	}
	
	
    protected:
	std::vector<Key>    m_keys;
	std::vector<double> m_dists;
	int                 m_nrKeysInserted;
    };
}

#endif 








