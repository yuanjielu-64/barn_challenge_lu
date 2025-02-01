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

#ifndef Antipatrea__ProximityQuery_HPP_
#define Antipatrea__ProximityQuery_HPP_

#include <cmath>

namespace Antipatrea
{
    template <typename Key>
    class ProximityQuery
    {
    public:
	ProximityQuery(void)
	{
	    m_k       = 0;
	    m_range   = INFINITY;	    
	}

	virtual ~ProximityQuery(void) 
	{
	}

	int GetNrNeighbors(void) const
	{
	    return m_k;
	}

	double GetRange(void)
	{
	    return m_range;
	}

	Key GetKey(void) const
	{
	    return m_key;
	}

	virtual void Clear(void)
	{
	}

	virtual void SetNrNeighbors(const int k)
	{
	    m_k = k;
	}


	virtual void SetRange(const double range)
	{
	    m_range = range;
	}

	virtual void SetKey(Key key)
	{
	    m_key = key;
	}

   protected:
	int    m_k;
	double m_range;
	Key    m_key;
   };
}

#endif 








