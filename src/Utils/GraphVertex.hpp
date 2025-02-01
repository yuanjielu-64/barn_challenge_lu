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

#ifndef Antipatrea__GraphVertex_HPP_
#define Antipatrea__GraphVertex_HPP_

#include "Utils/Vertex.hpp"
#include "Utils/DisjointSet.hpp"
#include <unordered_set>

namespace Antipatrea
{
template <typename Key>
class GraphVertex : public Vertex<Key>
{
  public:
	GraphVertex(void) : Vertex<Key>(),
						m_dsetElem(NULL)
	{
	}

	virtual ~GraphVertex(void)
	{
		if (m_dsetElem)
			delete m_dsetElem;
	}

	virtual const std::unordered_set<Key> *GetConnections(void) const
	{
		return &m_connections;
	}

	virtual std::unordered_set<Key> *GetConnections(void)
	{
		return &m_connections;
	}

	virtual const DisjointSet::Elem *GetDisjointSetElem(void) const
	{
		return m_dsetElem;
	}

	virtual DisjointSet::Elem *GetDisjointSetElem(void)
	{
		return m_dsetElem;
	}
	
	virtual void SetDisjointSetElem(DisjointSet::Elem *const dsetElem)
	{
		m_dsetElem = dsetElem;
	}

  protected:
	/**
	 *@brief Ids of the vertices to which it is connected.
	 */
	std::unordered_set<Key> m_connections;

	/**
	 *@brief Disjoint-set element which is used to keep track of the
	 *       connected components in the planner grapj.
	 */
	DisjointSet::Elem *m_dsetElem;
};
}

#endif
